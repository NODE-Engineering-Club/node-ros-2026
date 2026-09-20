"""Simulated battery.

The power panel has to answer one question honestly: *is there enough charge
left to finish the survey?* With a single sonar unit covering one side only, the
survey distance roughly doubles, so that question is not academic (brief,
section 7.5). Everything here exists to make that comparison testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class BatteryConfig:
    #: 6S LiFePO4-ish pack. Capacity PROVISIONAL — see docs/open_questions.md Q5.
    capacity_wh: float = 1200.0
    nominal_voltage: float = 25.2
    full_voltage: float = 29.4
    empty_voltage: float = 21.0
    #: Draw with everything on but no propulsion: Jetson, sonar, lidar, radios.
    hotel_load_w: float = 85.0
    #: Propulsion draw at full throttle.
    propulsion_max_w: float = 600.0
    internal_resistance_ohm: float = 0.035


@dataclass
class BatterySample:
    utc_ms: int
    voltage: float
    current: float
    power_w: float
    state_of_charge: float        # 0..1
    remaining_wh: float
    endurance_s: float            # at the recent average draw
    consumed_wh: float


class BatterySim:
    def __init__(self, config: BatteryConfig | None = None, initial_soc: float = 0.95) -> None:
        self.cfg = config or BatteryConfig()
        self.remaining_wh = self.cfg.capacity_wh * initial_soc
        self.consumed_wh = 0.0
        self._avg_power_w = self.cfg.hotel_load_w
        self._fault_multiplier = 1.0

    def set_load_multiplier(self, m: float) -> None:
        """Injectable fault: a fouled prop or a shorted cell draws harder."""
        self._fault_multiplier = m

    def step(self, dt: float, throttle: float) -> None:
        cfg = self.cfg
        # Propulsion power goes roughly as the cube of speed demand.
        prop = cfg.propulsion_max_w * (max(0.0, min(1.0, throttle)) ** 3)
        power = (cfg.hotel_load_w + prop) * self._fault_multiplier

        wh = power * dt / 3600.0
        self.remaining_wh = max(0.0, self.remaining_wh - wh)
        self.consumed_wh += wh
        # Exponential average over ~60 s, which is what an endurance estimate
        # should be based on rather than the instantaneous draw.
        alpha = min(1.0, dt / 60.0)
        self._avg_power_w += (power - self._avg_power_w) * alpha

    def sample(self, utc_ms: int) -> BatterySample:
        cfg = self.cfg
        soc = self.remaining_wh / cfg.capacity_wh if cfg.capacity_wh else 0.0
        # Open-circuit voltage: a flat middle with steep ends, as LiFePO4 does.
        ocv = cfg.empty_voltage + (cfg.full_voltage - cfg.empty_voltage) * (
            0.15 + 0.7 * soc + 0.15 * math.tanh((soc - 0.5) * 6.0) / math.tanh(3.0)
        ) / 1.0
        ocv = max(cfg.empty_voltage, min(cfg.full_voltage, ocv))

        current = self._avg_power_w / max(1.0, ocv)
        voltage = ocv - current * cfg.internal_resistance_ohm

        endurance = (
            self.remaining_wh / self._avg_power_w * 3600.0
            if self._avg_power_w > 1.0
            else float("inf")
        )
        return BatterySample(
            utc_ms=utc_ms,
            voltage=voltage,
            current=current,
            power_w=self._avg_power_w,
            state_of_charge=soc,
            remaining_wh=self.remaining_wh,
            endurance_s=endurance,
            consumed_wh=self.consumed_wh,
        )
