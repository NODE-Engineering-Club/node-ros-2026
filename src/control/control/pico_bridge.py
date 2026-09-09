"""
Pico Serial Bridge
Proprietaire unique du lien serie USB vers le Pico 2.

Fait 5 choses (un seul noeud car un seul owner de /dev/pico) :
  1. /control/effort (Twist) -> mixage skid-steer -> "L,R" serie
  2. Heartbeat : emet en CONTINU (timer). Commande fraiche -> "L,R",
     sinon -> "PING". Garde le lien "live" cote Pico (sinon il neutralise
     seul en AUTONOMOUS apres 500 ms de silence).
  3. /pico/mode_request (String) -> "CMD MODE <ESTOP|MANUAL|AUTONOMOUS>".
     La demande est REPETEE tant qu'elle est tenue : le firmware la laisse
     expirer apres SOFTWARE_REQUEST_TIMEOUT_MS sans rafraichissement.
     "RELEASE" arrete de la tenir (et donc la laisse expirer).
  4. Lit les lignes "[STAT] ..." du Pico (250 ms) -> /pico/status.
     Lit les accuses "[ACK] ..." -> /pico/command_ack.
     Lit les evenements (">>> ...", "[E-STOP] ...") -> /pico/events.
  5. Journalise une fois toute ligne serie qu'il ne sait pas classer.

Protocole firmware : USB CDC 115200, lignes '\n'.
  "L,R" norm -1..1 -> 1500 + v*500 us  (norm si |v|<=1.5 -> on clamp a +-1)
  Moteurs appliques cote Pico SEULEMENT si arme + AUTONOMOUS + 2s apres relais.

Le prefixe des lignes de statut
-------------------------------
Le firmware v3 ecrit "[STAT]". Ce noeud a longtemps cherche "STATE", donc
/pico/status ne publiait RIEN. Les deux prefixes sont acceptes desormais : la
derive entre les deux bouts d'un fil est exactement ce qui a casse ce chemin.

L'autorite reste a la radiocommande
-----------------------------------
Une demande logicielle ne peut que RESTREINDRE. Le firmware arbitre
(voir arbitrate_mode dans pico-node_v3.ino) et refuse ce qui serait moins
restrictif que Ch8. Ce noeud n'arbitre pas : il transmet et republie l'accuse.
"""
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

#: Modes acceptes sur /pico/mode_request. "AUTO" reste accepte : c'est ce que
#: publiaient les outils existants avant l'ajout d'ESTOP.
_MODE_WORDS = {
    "ESTOP": "ESTOP",
    "MANUAL": "MANUAL",
    "AUTO": "AUTONOMOUS",
    "AUTONOMOUS": "AUTONOMOUS",
}

#: Arrete de tenir la demande : le firmware la laisse expirer et rend la main
#: a Ch8. Bouton "Release to RC" cote GUI. Rendre la main est une action a part
#: entiere, distincte d'une demande de mode.
_RELEASE_WORDS = {"RELEASE", "NONE", "CLEAR"}

#: Prefixes des lignes de statut periodiques.
_STATUS_PREFIXES = ("[STAT]", "STATE")
#: Prefixes des lignes d'evenement (transitions, e-stop, commandes invalides).
_EVENT_PREFIXES = (">>>", "[E-STOP]", "Invalid cmd:", "[WARN]")
#: Bannieres de demarrage : connues, sans interet, ne pas les signaler.
_BANNER_PREFIXES = ("===", "Mode:")
#: Sauf celle-ci : elle dit si le retour de courant e-stop est compile ou non,
#: et c'est la seule fois ou le firmware le dit. La republier permet de la
#: capturer au lieu de la perdre au demarrage.
_FEEDBACK_BANNER = "E-STOP feedback:"


class PicoBridge(Node):
    def __init__(self):
        super().__init__("pico_bridge")

        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", BAUD_RATE)
        self.declare_parameter("send_hz", 20.0)     # >= 2 Hz requis (timeout Pico 500 ms)
        self.declare_parameter("cmd_timeout", 0.3)  # s : au-dela -> PING (le Pico neutralise seul)
        self.declare_parameter("yaw_invert", 1)     # +1/-1 si le bateau tourne a l'envers
        self.declare_parameter("thr_invert", 1)     # +1/-1 si avant/arriere inverses
        # Rafraichissement de la demande logicielle. Doit rester nettement sous
        # SOFTWARE_REQUEST_TIMEOUT_MS (5000 ms) cote firmware.
        self.declare_parameter("mode_refresh_s", 1.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._cmd_timeout = self.get_parameter("cmd_timeout").get_parameter_value().double_value
        self._yaw_inv = self.get_parameter("yaw_invert").get_parameter_value().integer_value
        self._thr_inv = self.get_parameter("thr_invert").get_parameter_value().integer_value
        self._mode_refresh_s = (
            self.get_parameter("mode_refresh_s").get_parameter_value().double_value
        )

        try:
            self._ser = serial.Serial(port, baud, timeout=0)  # non-bloquant
            self.get_logger().info(f"Pico connecte sur {port} a {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Impossible d'ouvrir {port}: {e}")
            self._ser = None

        # Derniere consigne recue + horodatage
        self._x = 0.0
        self._z = 0.0
        self._last_cmd = self.get_clock().now()
        self._rx = b""  # buffer de lecture serie

        # Demande de mode tenue, et quand elle a ete rafraichie pour la
        # derniere fois. None = aucune : le Pico suit Ch8 seul.
        self._held_mode = None
        self._last_mode_send = self.get_clock().now()

        # Lignes non classables deja signalees : on log une fois, pas a 20 Hz.
        self._warned_lines = set()

        self.create_subscription(Twist, "/control/effort", self._on_effort, 10)
        self.create_subscription(String, "/pico/mode_request", self._on_mode, 10)
        self._status_pub = self.create_publisher(String, "/pico/status", 10)
        self._ack_pub = self.create_publisher(String, "/pico/command_ack", 10)
        self._event_pub = self.create_publisher(String, "/pico/events", 10)

        hz = self.get_parameter("send_hz").get_parameter_value().double_value
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info("pico_bridge pret (heartbeat + modes + status + acks)")

    # --- Entrees ROS ---------------------------------------------------------
    def _on_effort(self, msg):
        self._x = msg.linear.x
        self._z = msg.angular.z
        self._last_cmd = self.get_clock().now()

    def _on_mode(self, msg):
        """Traduit une demande de mode en commande serie.

        Le noeud n'arbitre pas et ne prejuge pas du resultat : il envoie, et
        c'est l'accuse republie sur /pico/command_ack qui dit si le firmware a
        accepte. Afficher un mode parce qu'on l'a demande serait exactement la
        regle de securite 4 violee.
        """
        raw = msg.data.strip().upper()
        word = raw.replace("CMD ", "").replace("MODE ", "").strip()

        if word in _RELEASE_WORDS:
            if self._held_mode is not None:
                self.get_logger().info(
                    f"-> demande {self._held_mode} relachee ; "
                    "elle expirera cote Pico et Ch8 reprend la main"
                )
            self._held_mode = None
            return

        mode = _MODE_WORDS.get(word)
        if mode is None:
            self.get_logger().warn(f"mode_request inconnu : {msg.data!r}")
            return

        self._held_mode = mode
        self._send_mode_request()

    def _send_mode_request(self):
        if self._held_mode is None:
            return
        self._write(f"CMD MODE {self._held_mode}")
        self._last_mode_send = self.get_clock().now()

    # --- Boucle d'emission continue (coeur du heartbeat) ---------------------
    def _tick(self):
        self._drain()  # lire les lignes du Pico + vider le buffer RX

        # Rafraichir la demande tenue : sans cela le firmware la laisse expirer
        # au bout de 5 s et rend la main a Ch8.
        if self._held_mode is not None:
            since = (self.get_clock().now() - self._last_mode_send).nanoseconds / 1e9
            if since >= self._mode_refresh_s:
                self._send_mode_request()

        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if elapsed <= self._cmd_timeout:
            left, right = self._mix(self._x, self._z)
            self._write(f"{left:.3f},{right:.3f}")
        else:
            # Pas de consigne fraiche : on garde le lien vivant, le Pico met neutre seul.
            self._write("PING")

    # --- Mixage skid-steer (ton design : left = x + z) -----------------------
    def _mix(self, x, z):
        x *= self._thr_inv
        z *= self._yaw_inv
        left  = x + z
        right = x - z
        m = max(abs(left), abs(right), 1.0)  # normalise seulement si depasse 1
        left, right = left / m, right / m
        return (max(-1.0, min(1.0, left)), max(-1.0, min(1.0, right)))

    # --- Lecture serie : classe et republie ---------------------------------
    def _drain(self):
        if self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            if n:
                self._rx += self._ser.read(n)
            while b"\n" in self._rx:
                raw, self._rx = self._rx.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._classify(line)
        except serial.SerialException as e:
            self.get_logger().warn(f"Erreur lecture serie: {e}")

    def _classify(self, line):
        """Range une ligne serie dans un des quatre paniers, ou se plaint.

        Se plaindre importe : une ligne inconnue est en general le champ qui
        compte, ou le signe que le firmware a change de format sous nos pieds.
        C'est ce silence-la qui a laisse /pico/status vide.
        """
        if line.startswith(_STATUS_PREFIXES):
            self._status_pub.publish(String(data=line))
            return

        if line.startswith("[ACK]"):
            self._ack_pub.publish(String(data=line))
            # Un refus doit etre visible dans les logs du bateau, pas seulement
            # dans l'interface : c'est la trace de ce que le firmware a refuse.
            if " rejected" in line:
                self.get_logger().warn(f"Pico a refuse une commande : {line}")
            else:
                self.get_logger().info(f"Pico : {line}")
            return

        if line.startswith(_EVENT_PREFIXES):
            self._event_pub.publish(String(data=line))
            self.get_logger().info(f"Pico : {line}")
            return

        if line.startswith(_FEEDBACK_BANNER):
            self._event_pub.publish(String(data=line))
            self.get_logger().info(f"Pico : {line}")
            return

        if line.startswith(_BANNER_PREFIXES):
            return

        if line not in self._warned_lines:
            self._warned_lines.add(line)
            # Borne : un firmware qui deverse du texte varie ne doit pas faire
            # grossir ce set indefiniment.
            if len(self._warned_lines) > 50:
                self._warned_lines.clear()
            self.get_logger().warn(f"Ligne serie non classee : {line!r}")

    # --- Ecriture serie ------------------------------------------------------
    def _write(self, s):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            self._ser.write((s + "\n").encode())
        except serial.SerialException as e:
            self.get_logger().warn(f"Erreur ecriture serie: {e}")

    def destroy_node(self):
        self._write("0,0")  # neutre a la sortie
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
