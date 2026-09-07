"""Simulated data source.

Drives the whole backend from ``asket_sim`` in-process, with no ROS. This is
what runs when a developer opens the GUI on a laptop, and it is also what the
backend's own tests use.

It is not a stub. Everything the real source produces, this produces: the same
stream names, the same payload shapes, the same command surface. The one thing
it adds is a command surface for **fault injection**, so the degraded cases can
be reached from the GUI itself rather than only from a test.
"""

from __future__ import annotations

import math
import time

from asket_common.heading import (
    SOURCE_GNSS_COMPASS,
    SOURCE_MAGNETOMETER,
    evaluate_heading,
)
from asket_common.survey import swath_half_width_m
from asket_sim.core.faults import FAULTS
from asket_sim.core.pico import MODE_VALUES
from asket_sim.core.vessel import HEADING_SOURCE_GNSS
from asket_sim.core.world import SimWorld, WorldConfig
from omniscan_bridge.core.status import SonarHealthTracker

from . import payloads
from .commands import (
    CMD_CLEAR_FAULT,
    CMD_CUT_PROPULSION,
    CMD_INJECT_FAULT,
    CMD_SET_MODE,
    CMD_SET_PING_PARAMETERS,
)
from .source import CommandOutcome, Sample
from .streams import DETAIL_FULL


class SimSource:
    """A :class:`~gui_backend.core.source.DataSource` backed by ``SimWorld``."""

    def __init__(
        self,
        world: SimWorld | None = None,
        time_scale: float = 1.0,
        max_step_s: float = 0.2,
    ) -> None:
        self.world = world or SimWorld(WorldConfig())
        self.time_scale = time_scale
        #: Cap on a single step. If the process is descheduled for a second we
        #: simulate a second of survey, not a second of frozen vessel — but we
        #: do it in bounded chunks so the physics stays sane.
        self.max_step_s = max_step_s

        self._last_wall_s: float | None = None
        self._sonar_health = SonarHealthTracker()
        self._sonar_health.on_parameters_commanded(
            self.world.cfg.sonar.range_setting_m,
            self.world.cfg.sonar.gain,
            self.world.cfg.sonar.ping_rate_hz,
        )
        self._pings_seen = 0
        self._track: list[tuple[float, float]] = []
        self._coverage: list[dict] = []
        self._last_track_utc = 0

    # -- clock ------------------------------------------------------------

    def step(self, now_s: float | None = None) -> None:
        now_s = now_s if now_s is not None else time.monotonic()
        if self._last_wall_s is None:
            self._last_wall_s = now_s
            return

        elapsed = (now_s - self._last_wall_s) * self.time_scale
        self._last_wall_s = now_s

        remaining = min(elapsed, 5.0)   # never try to catch up more than 5 s
        while remaining > 1e-4:
            dt = min(self.max_step_s, remaining)
            self.world.step(dt)
            remaining -= dt

        self._consume_sonar()
        self._accumulate_map_layers()

    def now_utc_ms(self) -> int:
        return self.world.utc_ms

    # -- derived state ----------------------------------------------------

    def _consume_sonar(self) -> None:
        """Feed the sonar health tracker as though the bridge were running.

        In the full sim stack the real bridge does this over a real socket; the
        backend's own sim mode short-circuits it so the sonar panel is alive on
        a laptop with nothing else running.
        """
        pings = self.world.take_pings()
        if not pings:
            return
        period = 1.0 / max(0.1, self.world.cfg.sonar.ping_rate_hz)
        # Space the batch across the interval it was actually produced in, so
        # the measured rate is the rate the sonar ran at rather than the rate
        # this loop happened to drain it.
        first_t = self.world.t - period * (len(pings) - 1)
        for i, ping in enumerate(pings):
            self._pings_seen += 1
            valid = sum(1 for p in ping.points if p.pt_type != 0)
            self._sonar_health.on_point_set(
                utc_msec=ping.utc_ms,
                num_points=len(ping.points),
                num_valid=valid,
                speed_of_sound=ping.speed_of_sound_ms,
                # Measured against SIMULATED time, not the wall clock. With
                # time_scale > 1 the wall clock would report a ping rate the
                # vessel is not achieving, which is the opposite of the point.
                now_monotonic=first_t + i * period,
                now_utc_ms=ping.utc_ms - self.world.cfg.sonar.clock_offset_ms,
            )
        snap = self.world.snapshot()
        self._sonar_health.on_attitude(snap.sonar_pitch_deg, snap.sonar_roll_deg)

    def _accumulate_map_layers(self) -> None:
        """Track history and the coverage ribbon.

        Coverage is accumulated as a ribbon of near/far edge pairs rather than a
        filled polygon: a gap in the data then shows as a gap in the ribbon,
        which is precisely what an operator with one single-sided sonar needs to
        see (brief, section 7.1).
        """
        snap = self.world.snapshot()
        v = snap.vessel
        if snap.utc_ms - self._last_track_utc < 1000:
            return
        self._last_track_utc = snap.utc_ms

        self._track.append((round(v.lat, 7), round(v.lon, 7)))
        del self._track[: max(0, len(self._track) - 4000)]

        # Only paint coverage where the sonar is actually ensonifying: on a
        # survey leg, pinging, and with a valid heading. Painting during a turn
        # or a heading dropout would claim coverage we do not have.
        painting = (
            v.on_survey
            and v.heading_valid
            and self.world.sonar.pinging
            and not self.world.faults.active("sonar_dropout")
        )
        if not painting:
            self._coverage.append({"gap": True})
            del self._coverage[: max(0, len(self._coverage) - 4000)]
            return

        half = swath_half_width_m(
            self.world.cfg.sonar.range_setting_m,
            self.world.cfg.sonar.mounting_tilt_deg,
            snap.seabed_depth_m,
        )
        nadir = snap.seabed_depth_m * math.tan(
            max(0.0, math.radians(self.world.cfg.sonar.mounting_tilt_deg - 30.0))
        )
        self._coverage.append(
            {
                "lat": round(v.lat, 7),
                "lon": round(v.lon, 7),
                "heading_deg": round(v.heading_deg, 1),
                "half_width_m": round(half, 1),
                "inner_gap_m": round(nadir, 1),
                "side": self.world.cfg.sonar_side,
            }
        )
        del self._coverage[: max(0, len(self._coverage) - 4000)]

    def _heading_estimate(self):
        v = self.world.snapshot().vessel
        source = (
            SOURCE_GNSS_COMPASS
            if self.world.cfg.vessel.heading_source == HEADING_SOURCE_GNSS
            else SOURCE_MAGNETOMETER
        )
        return evaluate_heading(
            heading_deg=v.heading_deg,
            source=source,
            cog_deg=v.cog_deg,
            sog_ms=v.sog_ms,
            reported_accuracy_deg=v.heading_accuracy_deg,
            source_valid=v.heading_valid,
        )

    def _survey_remaining_m(self) -> float:
        plan = self.world.plan
        done = self.world.vessel.distance_travelled
        return max(0.0, plan.total_track_distance_m() - done)

    # -- the DataSource interface -----------------------------------------

    def snapshot(self, stream: str, detail: str = DETAIL_FULL) -> Sample | None:
        snap = self.world.snapshot()
        utc = snap.utc_ms

        if stream == "vessel":
            return Sample(stream, utc, payloads.vessel_payload(snap.vessel, detail))
        if stream == "heading":
            return Sample(stream, utc, payloads.heading_payload(self._heading_estimate(), detail))
        if stream == "pico":
            return Sample(stream, utc, payloads.pico_payload(snap.pico, detail))
        if stream == "power":
            return Sample(
                stream,
                utc,
                payloads.power_payload(
                    snap.battery, detail,
                    survey_remaining_m=self._survey_remaining_m(),
                    speed_ms=snap.vessel.sog_ms,
                ),
            )
        if stream == "sonar":
            health = self._sonar_health.health(
                connected=self.world.sonar.pinging
                and not self.world.faults.active("sonar_dropout"),
                packet_loss_ratio=0.0,
                packets_parsed=self._pings_seen,
                checksum_errors=0,
                bytes_discarded=0,
                seconds_since_data=0.0
                if not self.world.faults.active("sonar_dropout")
                else 30.0,
                now_monotonic=self.world.t,
            )
            return Sample(stream, utc, payloads.sonar_payload(health, detail))
        if stream == "lidar":
            if snap.lidar is None:
                return None
            decimation = {"full": 1, "reduced": 4, "minimal": 12}[detail]
            return Sample(
                stream, snap.lidar.utc_ms,
                payloads.lidar_payload(snap.lidar, detail, decimation),
            )
        if stream == "link":
            # The bearer-side facts only. Profile, client count and the rate
            # actually being pushed are the hub's to know, and it merges them
            # in — the source has no idea how many browsers are attached.
            return Sample(
                stream, utc,
                payloads.link_payload(
                    snap.link, profile="", profile_manual=False, clients=0,
                    rate_bytes_per_s=0.0, detail=detail,
                ),
            )
        if stream == "mission":
            return Sample(stream, utc, dict(self.state()["mission"]))
        if stream == "plan":
            return Sample(stream, utc, payloads.plan_payload(self.world.plan))
        if stream == "track":
            return Sample(stream, utc, {"points": [[lon, lat] for lat, lon in self._track]})
        if stream == "coverage":
            return Sample(stream, utc, {"segments": list(self._coverage)})
        return None

    def state(self) -> dict:
        """Full-detail everything, for alarms and command confirmation."""
        snap = self.world.snapshot()
        heading = self._heading_estimate()
        power = payloads.power_payload(
            snap.battery, DETAIL_FULL,
            survey_remaining_m=self._survey_remaining_m(),
            speed_ms=snap.vessel.sog_ms,
        )
        sonar_sample = self.snapshot("sonar", DETAIL_FULL)
        return {
            "utc_ms": snap.utc_ms,
            "vessel": payloads.vessel_payload(snap.vessel, DETAIL_FULL),
            "heading": payloads.heading_payload(heading, DETAIL_FULL),
            "pico": payloads.pico_payload(snap.pico, DETAIL_FULL),
            "power": power,
            "sonar": sonar_sample.payload if sonar_sample else {},
            "mission": {"state": "IDLE"},
            "link_sample": snap.link,
            "disk_free_bytes": snap.disk_free_bytes,
            "disk_total_bytes": snap.disk_total_bytes,
            "active_faults": snap.active_faults,
        }

    def alarm_state(self) -> dict:
        """The flattened view :mod:`gui_backend.core.alarms` expects."""
        state = self.state()
        return {
            # In sim the source is in-process, so vessel data is never stale.
            # The link between the browser and the backend is a separate thing
            # and the client reports on that itself.
            "link_age_s": 0.0,
            "state_of_charge": state["power"].get("state_of_charge"),
            "can_finish_survey": state["power"].get("can_finish_survey"),
            "recording": state["mission"].get("state") == "RECORDING",
            "disk_free_bytes": state["disk_free_bytes"],
            "sonar_expected": True,
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
        if name == CMD_SET_MODE:
            mode = MODE_VALUES.get(str(args.get("mode", "")).upper())
            if mode is None:
                return CommandOutcome(False, f"unknown mode {args.get('mode')!r}")
            return CommandOutcome(self.world.pico.request_mode(mode), "sent to the Pico")

        if name == CMD_CUT_PROPULSION:
            return CommandOutcome(self.world.pico.request_mode(0), "sent to the Pico")

        if name == CMD_SET_PING_PARAMETERS:
            cfg = self.world.cfg.sonar
            cfg.range_setting_m = float(args.get("range_m", cfg.range_setting_m))
            cfg.gain = int(args.get("gain", cfg.gain))
            cfg.ping_rate_hz = max(0.1, float(args.get("ping_rate_hz", cfg.ping_rate_hz)))
            self._sonar_health.on_parameters_commanded(
                cfg.range_setting_m, cfg.gain, cfg.ping_rate_hz
            )
            return CommandOutcome(True, "sent to the sonar")

        if name == CMD_INJECT_FAULT:
            try:
                self.world.inject_fault(
                    str(args.get("fault", "")),
                    float(args["duration_s"]) if args.get("duration_s") else None,
                )
            except (KeyError, ValueError) as exc:
                return CommandOutcome(False, str(exc))
            return CommandOutcome(True, f"injected {args.get('fault')}")

        if name == CMD_CLEAR_FAULT:
            fault = str(args.get("fault", ""))
            if fault in ("*", "all", ""):
                self.world.faults.clear_all()
            else:
                self.world.clear_fault(fault)
            return CommandOutcome(True, "cleared")

        return CommandOutcome(False, f"command {name!r} is not available in sim")

    def describe(self) -> dict:
        cfg = self.world.cfg
        return {
            "mode": "sim",
            "origin": {"lat": cfg.origin_lat, "lon": cfg.origin_lon},
            "sonar_side": cfg.sonar_side,
            "available_faults": sorted(FAULTS),
        }
