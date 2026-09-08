"""Real data source: ROS 2 topics.

The counterpart to :class:`~gui_backend.core.sim_source.SimSource`, satisfying
the same interface so that everything above it — the hub, the payloads, the
alarms, the command lifecycle, the frontend — is identical in both modes.

It holds the latest message from each configured topic and builds payloads on
demand, rather than converting on every callback. Most messages arrive far
faster than any client is subscribed at, and converting a 10 Hz IMU message
into JSON forty times a second to send it twice would be work done for nothing.

``rclpy`` is imported lazily so this module can be *imported* without ROS
present, which keeps the adapters importable in tests.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from asket_common.heading import (
    SOURCE_GNSS_COMPASS,
    SOURCE_MAGNETOMETER,
    SOURCE_NONE,
    evaluate_heading,
)

from . import adapters, payloads
from .commands import (
    CMD_CUT_PROPULSION,
    CMD_SET_MODE,
    CMD_SET_PING_PARAMETERS,
    CMD_START_MISSION,
    CMD_STOP_MISSION,
)
from .source import CommandOutcome, Sample
from .streams import DETAIL_FULL

MODE_NAMES = {0: "ESTOP", 1: "MANUAL", 2: "AUTONOMOUS"}


@dataclass
class LatestMessage:
    message: object | None = None
    received_monotonic: float = 0.0

    @property
    def age_s(self) -> float:
        if self.received_monotonic == 0.0:
            return float("inf")
        return time.monotonic() - self.received_monotonic


class RosSource:
    """Reads real topics. Constructed with an already-created ``rclpy`` node."""

    def __init__(self, node, config: dict) -> None:
        self.node = node
        self.config = config
        self.latest: dict[str, LatestMessage] = {}
        self._track: list[tuple[float, float]] = []
        self._coverage: list[dict] = []
        self._last_track_utc = 0
        self._publishers: dict[str, object] = {}
        self._service_clients: dict[str, object] = {}
        self._link = adapters.link_from_measurements("wifi", 1.0, 0.0, 800_000.0)

        self._subscribe_all()
        self._prepare_commands()

    # -- wiring -----------------------------------------------------------

    def _subscribe_all(self) -> None:
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        for name, spec in (self.config.get("sources") or {}).items():
            self.latest[name] = LatestMessage()
            try:
                msg_type = get_message(spec["type"])
            except (ImportError, AttributeError, ValueError) as exc:
                # A missing optional type is a degraded GUI, not a dead one.
                level = self.node.get_logger().warn if spec.get("optional") else \
                    self.node.get_logger().error
                level(f"cannot resolve {spec['type']} for {name}: {exc}")
                continue

            self.node.create_subscription(
                msg_type,
                spec["topic"],
                self._make_callback(name),
                sensor_qos,
            )

    def _make_callback(self, name: str):
        def callback(msg) -> None:
            self.latest[name] = LatestMessage(msg, time.monotonic())

        return callback

    def _prepare_commands(self) -> None:
        from rosidl_runtime_py.utilities import get_message, get_service

        for name, spec in (self.config.get("commands") or {}).items():
            try:
                if "topic" in spec:
                    self._publishers[name] = self.node.create_publisher(
                        get_message(spec["type"]), spec["topic"], 10
                    )
                else:
                    self._service_clients[name] = self.node.create_client(
                        get_service(spec["type"]), spec["service"]
                    )
            except (ImportError, AttributeError, ValueError) as exc:
                self.node.get_logger().error(f"cannot prepare command {name}: {exc}")

    # -- clock ------------------------------------------------------------

    def step(self, now_s: float) -> None:
        """Nothing to advance: ROS callbacks fill the buffers themselves."""
        self._accumulate_map_layers()

    def now_utc_ms(self) -> int:
        return int(time.time() * 1000)

    # -- assembly ---------------------------------------------------------

    def _msg(self, name: str):
        entry = self.latest.get(name)
        return entry.message if entry else None

    def _vessel_record(self):
        compass = self._msg("compass_heading")
        heading_deg = float(compass.data) if compass is not None else None
        fix = self._msg("gnss_fix")
        if fix is None:
            return None

        # Heading is invalid if its topic has gone quiet, even though the last
        # value is still in memory. A frozen heading that still renders is
        # exactly the failure this whole design is built to prevent.
        heading_age = self.latest.get("compass_heading", LatestMessage()).age_s
        heading_valid = heading_deg is not None and heading_age < 2.0

        expected = (self.config.get("heading") or {}).get("expected_source", "magnetometer")
        source = SOURCE_GNSS_COMPASS if expected == "gnss_compass" else SOURCE_MAGNETOMETER

        return adapters.vessel_from_ros(
            fix,
            heading_deg,
            self._msg("gps_velocity"),
            self._msg("imu"),
            self._msg("gps_raw"),
            extra={
                "heading_source": source if heading_valid else SOURCE_NONE,
                "heading_valid": heading_valid,
                "distance_travelled_m": self._track_distance_m(),
            },
        )

    def _heading_estimate(self):
        vessel = self._vessel_record()
        if vessel is None:
            return evaluate_heading(None, SOURCE_NONE, 0.0, 0.0, source_valid=False)
        return evaluate_heading(
            heading_deg=vessel.heading_deg,
            source=vessel.heading_source,
            cog_deg=vessel.cog_deg,
            sog_ms=vessel.sog_ms,
            reported_accuracy_deg=vessel.heading_accuracy_deg,
            source_valid=vessel.heading_valid,
        )

    def _track_distance_m(self) -> float:
        from asket_common.geo import LocalOrigin

        if len(self._track) < 2:
            return 0.0
        origin = LocalOrigin(*self._track[0])
        total = 0.0
        previous = origin.to_enu(*self._track[0])
        for lat, lon in self._track[1:]:
            current = origin.to_enu(lat, lon)
            total += math.hypot(current[0] - previous[0], current[1] - previous[1])
            previous = current
        return total

    def _accumulate_map_layers(self) -> None:
        vessel = self._vessel_record()
        if vessel is None:
            return
        utc = vessel.utc_ms
        if utc - self._last_track_utc < 1000:
            return
        self._last_track_utc = utc
        self._track.append((round(vessel.lat, 7), round(vessel.lon, 7)))
        del self._track[: max(0, len(self._track) - 4000)]

    def set_link_measurement(self, active_link: str, quality: float, rtt_ms: float,
                             capacity: float) -> None:
        """Fed by the node from whatever link telemetry it has."""
        self._link = adapters.link_from_measurements(active_link, quality, rtt_ms, capacity)

    # -- the DataSource interface -----------------------------------------

    def snapshot(
        self, stream: str, detail: str = DETAIL_FULL, cursor: int = 0
    ) -> Sample | None:
        if stream == "vessel":
            record = self._vessel_record()
            return Sample(stream, record.utc_ms, payloads.vessel_payload(record, detail)) \
                if record else None

        if stream == "heading":
            record = self._vessel_record()
            if record is None:
                return None
            return Sample(
                stream, record.utc_ms,
                payloads.heading_payload(self._heading_estimate(), detail),
            )

        if stream == "pico":
            msg = self._msg("pico_status")
            if msg is None:
                return None
            record = adapters.pico_from_ros(msg)
            return Sample(stream, record.utc_ms, payloads.pico_payload(record, detail))

        if stream == "power":
            msg = self._msg("battery")
            if msg is None:
                return None
            vessel_cfg = self.config.get("vessel") or {}
            record = adapters.battery_from_ros(
                msg,
                capacity_wh=vessel_cfg.get("battery_capacity_wh", 1200.0),
                hotel_load_w=vessel_cfg.get("hotel_load_w", 85.0),
            )
            vessel = self._vessel_record()
            return Sample(
                stream, record.utc_ms,
                payloads.power_payload(
                    record, detail, speed_ms=vessel.sog_ms if vessel else None
                ),
            )

        if stream == "sonar":
            msg = self._msg("sonar_status")
            if msg is None:
                return None
            record = adapters.sonar_from_ros(msg)
            return Sample(stream, self.now_utc_ms(), payloads.sonar_payload(record, detail))

        if stream == "lidar":
            msg = self._msg("lidar_scan")
            if msg is None:
                return None
            record = adapters.lidar_from_ros(msg)
            decimation = {"full": 1, "reduced": 4, "minimal": 12}[detail]
            return Sample(
                stream, record.utc_ms, payloads.lidar_payload(record, detail, decimation)
            )

        if stream == "link":
            return Sample(
                stream, self.now_utc_ms(),
                payloads.link_payload(
                    self._link, profile="", profile_manual=False, clients=0,
                    rate_bytes_per_s=0.0, detail=detail,
                ),
            )

        if stream == "track":
            step = {"full": 1, "reduced": 3, "minimal": 10}[detail]
            return Sample(
                stream, self.now_utc_ms(),
                payloads.track_payload(self._track, cursor, step),
                cursor=len(self._track),
            )

        if stream == "coverage":
            step = {"full": 1, "reduced": 3, "minimal": 10}[detail]
            side = (self.config.get("survey") or {}).get("sonar_side", "starboard")
            return Sample(
                stream, self.now_utc_ms(),
                payloads.coverage_payload(self._coverage, cursor, side, step),
                cursor=len(self._coverage),
            )

        return None

    def state(self) -> dict:
        vessel = self._vessel_record()
        pico_msg = self._msg("pico_status")
        sonar = self.snapshot("sonar", DETAIL_FULL)
        power = self.snapshot("power", DETAIL_FULL)
        return {
            "utc_ms": self.now_utc_ms(),
            "vessel": payloads.vessel_payload(vessel, DETAIL_FULL) if vessel else {},
            "heading": payloads.heading_payload(self._heading_estimate(), DETAIL_FULL),
            "pico": payloads.pico_payload(adapters.pico_from_ros(pico_msg), DETAIL_FULL)
            if pico_msg
            else {},
            "power": power.payload if power else {},
            "sonar": sonar.payload if sonar else {},
            "mission": {"state": "IDLE"},
            "link_sample": self._link,
        }

    def alarm_state(self) -> dict:
        state = self.state()
        pico_age = self.latest.get("pico_status", LatestMessage()).age_s
        return {
            # Age of the freshest thing the vessel sends us: if that has gone
            # quiet, everything on screen is stale regardless of the browser's
            # own connection being fine.
            "link_age_s": min(
                pico_age, self.latest.get("gnss_fix", LatestMessage()).age_s
            ),
            "state_of_charge": state["power"].get("state_of_charge"),
            "can_finish_survey": state["power"].get("can_finish_survey"),
            "recording": state["mission"].get("state") == "RECORDING",
            "disk_free_bytes": None,
            "sonar_expected": self._msg("sonar_status") is not None,
            "sonar_connected": state["sonar"].get("connected"),
            "sonar_seconds_since_data": state["sonar"].get("seconds_since_data"),
            "clock_offset_ms": state["sonar"].get("clock_offset_ms"),
            "heading_valid": state["heading"].get("valid"),
            "heading_divergence_suspicious": state["heading"].get("divergence_suspicious"),
            "heading_divergence_deg": state["heading"].get("divergence_deg"),
            "roll_deg": state["vessel"].get("roll_deg"),
            "geofence_distance_m": None,
            "rc_link_ok": state["pico"].get("rc_link_ok"),
        }

    def send_command(self, name: str, args: dict) -> CommandOutcome:
        if name in (CMD_SET_MODE, CMD_CUT_PROPULSION):
            publisher = self._publishers.get("mode_request")
            if publisher is None:
                return CommandOutcome(False, "no mode_request publisher configured")
            from std_msgs.msg import String

            mode = "ESTOP" if name == CMD_CUT_PROPULSION else str(args.get("mode", "")).upper()
            if mode not in MODE_NAMES.values():
                return CommandOutcome(False, f"unknown mode {mode!r}")
            publisher.publish(String(data=mode))
            return CommandOutcome(True, "sent to the Pico")

        if name == CMD_SET_PING_PARAMETERS:
            return self._call_service(
                "set_ping_parameters",
                lambda req: (
                    setattr(req, "range_m", float(args.get("range_m", 30.0))),
                    setattr(req, "gain", int(args.get("gain", 4))),
                    setattr(req, "ping_rate_hz", float(args.get("ping_rate_hz", 5.0))),
                ),
            )

        if name == CMD_START_MISSION:
            return self._call_service(
                "start_mission",
                lambda req: (
                    setattr(req, "name", str(args.get("name", "mission"))),
                    setattr(req, "record_rosbag", bool(args.get("record_rosbag", False))),
                ),
            )

        if name == CMD_STOP_MISSION:
            return self._call_service("stop_mission", lambda req: None)

        return CommandOutcome(False, f"command {name!r} is not wired up")

    def _call_service(self, key: str, fill) -> CommandOutcome:
        """Send a service request without waiting for it.

        Blocking here would stall the hub and therefore every client. The
        response does not decide anything anyway: confirmation comes from
        observing the vessel's own status, never from an acknowledgement that a
        request was received.
        """
        client = self._service_clients.get(key)
        if client is None:
            return CommandOutcome(False, f"no client configured for {key}")
        if not client.service_is_ready():
            return CommandOutcome(False, f"{key} service is not available")
        request = client.srv_type.Request()
        fill(request)
        client.call_async(request)
        return CommandOutcome(True, "sent")

    def describe(self) -> dict:
        return {
            "mode": "ros",
            "topics": {
                name: spec.get("topic") or spec.get("service")
                for name, spec in (self.config.get("sources") or {}).items()
            },
        }
