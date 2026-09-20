"""
Pico Serial Bridge
Proprietaire unique du lien serie USB vers le Pico 2.

Fait 4 choses (un seul noeud car un seul owner de /dev/pico) :
  1. /control/effort (Twist) -> mixage skid-steer -> "L,R" serie
  2. Heartbeat : emet en CONTINU (timer). Commande fraiche -> "L,R",
     sinon -> "PING". Garde le lien "live" cote Pico (sinon il retombe
     seul en MANUAL en zone Ch8 HIGH apres 600 ms de silence).
  3. /pico/mode_request (String "AUTO"/"MANUAL") -> "MODE AUTO"/"MODE MANUAL"
     (Foxglove = GUI du quai publie ce topic).
  4. Lit les lignes "STATE ..." du Pico (250 ms) -> /pico/status,
     et COMPTE celles qu'il rejette (voir plus bas : c'est le point).

Protocole firmware : pico-node_v4. USB CDC 115200, lignes '\n'.
  "L,R" norm -1..1 -> 1500 + v*500 us  (norm si |v|<=1.5 -> on clamp a +-1)
  Moteurs appliques cote Pico SEULEMENT si arme + AUTONOMOUS + 2s apres relais.

POURQUOI ON COMPTE LES LIGNES REJETEES
--------------------------------------
Le filtre ci-dessous a deja tout jete pendant des mois, en silence. Le firmware
flashe (v3) emettait "[STAT] ...", ce noeud cherchait "STATE...", aucune ligne
ne passait, /pico/status ne publiait jamais rien -- et comme le sens descendant
marchait (le bateau bougeait), rien n'avait l'air casse. Personne n'ecoutait ce
topic, donc personne n'a rien vu.

Un compteur et un log auraient transforme des mois d'enquete en une ligne de
journal au premier demarrage. C'est tout ce que fait le code ajoute ici : il ne
rend PAS le noeud multi-format (un seul format, pico-node_v4, delibere -- deux
formats acceptes veut dire qu'un des deux n'est jamais teste). C'est un
detecteur : le jour ou quelqu'un flashe autre chose, il le dit tout de suite.
"""
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200


class PicoBridge(Node):
    def __init__(self):
        super().__init__("pico_bridge")

        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", BAUD_RATE)
        self.declare_parameter("send_hz", 20.0)     # >= 2 Hz requis (timeouts Pico 500/600 ms)
        self.declare_parameter("cmd_timeout", 0.3)  # s : au-dela -> PING (le Pico neutralise seul)
        self.declare_parameter("yaw_invert", 1)     # +1/-1 si le bateau tourne a l'envers
        self.declare_parameter("thr_invert", 1)     # +1/-1 si avant/arriere inverses

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._cmd_timeout = self.get_parameter("cmd_timeout").get_parameter_value().double_value
        self._yaw_inv = self.get_parameter("yaw_invert").get_parameter_value().integer_value
        self._thr_inv = self.get_parameter("thr_invert").get_parameter_value().integer_value

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

        # Detecteur de desaccord de format. Voir le docstring du module.
        self._lines_kept = 0
        self._lines_rejected = 0
        self._rejected_sample = ""
        self._warned_rejecting = False
        self._fw_version = None

        self.create_subscription(Twist, "/control/effort", self._on_effort, 10)
        self.create_subscription(String, "/pico/mode_request", self._on_mode, 10)
        self._status_pub = self.create_publisher(String, "/pico/status", 10)

        hz = self.get_parameter("send_hz").get_parameter_value().double_value
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info("pico_bridge pret (heartbeat + modes + status)")

    # --- Entrees ROS ---------------------------------------------------------
    def _on_effort(self, msg):
        self._x = msg.linear.x
        self._z = msg.angular.z
        self._last_cmd = self.get_clock().now()

    def _on_mode(self, msg):
        v = msg.data.strip().upper().replace("MODE ", "")
        if v == "AUTO":
            self._write("MODE AUTO")
            self.get_logger().info("-> MODE AUTO envoye au Pico")
        elif v == "MANUAL":
            self._write("MODE MANUAL")
            self.get_logger().info("-> MODE MANUAL envoye au Pico")
        else:
            self.get_logger().warn(f"mode_request inconnu : {msg.data!r}")

    # --- Boucle d'emission continue (coeur du heartbeat) ---------------------
    def _tick(self):
        self._drain()  # lire le STATE du Pico + vider le buffer RX

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

    # --- Lecture serie : republie les lignes STATE ---------------------------
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
                if not line:
                    continue
                if line.startswith("STATE"):
                    self._lines_kept += 1
                    self._status_pub.publish(String(data=line))
                    continue
                if line.startswith("[VER] "):
                    self._on_version_line(line)
                    continue
                # Tout le reste : [ACK], "Invalid cmd:", bruit serie. Compte,
                # pas jete en silence.
                self._lines_rejected += 1
                if not self._rejected_sample:
                    self._rejected_sample = line[:80]
            self._check_format()
        except serial.SerialException as e:
            self.get_logger().warn(f"Erreur lecture serie: {e}")

    # --- Detection de desaccord de format ------------------------------------
    def _on_version_line(self, line):
        """Le Pico annonce son identite au demarrage : "[VER] pico-node 4"."""
        parts = line.split()
        version = parts[-1] if len(parts) >= 2 else ""
        if version != self._fw_version:
            self._fw_version = version
            self.get_logger().info(f"Pico firmware : {' '.join(parts[1:])}")

    def _check_format(self):
        """Crie une fois si le Pico parle et qu'on ne comprend rien.

        Le seuil (20 lignes rejetees, zero gardee) correspond a environ cinq
        secondes de STATE a 4 Hz : assez pour ne pas crier sur un fragment de
        ligne au demarrage, assez peu pour que ce soit dit avant la mise a
        l'eau.
        """
        if self._warned_rejecting or self._lines_kept:
            return
        if self._lines_rejected < 20:
            return
        self._warned_rejecting = True
        self.get_logger().error(
            f"Le Pico emet ({self._lines_rejected} lignes) mais AUCUNE ne "
            f"commence par 'STATE' : /pico/status ne publiera rien. "
            f"Exemple : {self._rejected_sample!r}. "
            f"Firmware attendu : pico-node_v4. Un firmware plus ancien "
            f"(v3) emet '[STAT] Mode:...' et n'est pas supporte."
        )

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
