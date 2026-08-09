"""BMS serial reader.

Owns the serial link to the battery management system (connected via an
MCP2221 USB-UART bridge) and republishes its periodic text report as ROS
topics: a standard sensor_msgs/BatteryState (voltage/cells/temp/current —
plottable, gauge-able, but no dedicated "battery" panel in Foxglove), a
diagnostic_msgs/DiagnosticArray on /diagnostics (renders natively in
Foxglove's Diagnostics panel — a colored OK/ERROR summary plus every field
as key/value, ERROR if a fault flag trips or DSG drops), and a raw string
with everything the BMS reports verbatim.

Expected report, one block every period (no blank line between blocks —
each starts with its own "--- BQ76920 [t=... ms] ---" header line):
    --- BQ76920 [t=41254 ms] ---
    Cells: 3.999  3.987  3.986  4.000  V
    Sum:   3.999  7.986  11.972  15.972  V (running sum, last~=Pack)
    Pack:  15.98 V
    Temp:  -76.4 C (ext NTC)
    Curr:  0.02 A
    FETs:  CHG=ON DSG=ON   [OUTPUT LIVE]
    Flags: OV=0 UV=0 OCD=0 SCD=0 XREADY=0

"Sum" is redundant with "Cells" (a running total) and isn't republished as
a separate field, but is included in the raw text. "Flags" (fault bits)
has no home in sensor_msgs/BatteryState either, so it's raw-text-only too
— a non-zero flag is a real fault condition worth having in the log even
without a dedicated topic for it. The block is flushed on "Flags:", the
last field the firmware sends each cycle — flushing on "FETs:" (the
previous field) would silently drop Flags off of every published block.
"""
import re

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyACM3"
BAUD_RATE = 115200

_CELLS_RE = re.compile(r"Cells:\s*([\d.\s-]+?)\s*V")
_PACK_RE = re.compile(r"Pack:\s*(-?[\d.]+)\s*V")
_TEMP_RE = re.compile(r"Temp:\s*(-?[\d.]+)\s*C")
_CURR_RE = re.compile(r"Curr:\s*(-?[\d.]+)\s*A")
_FETS_RE = re.compile(r"FETs:\s*CHG=(ON|OFF)\s*DSG=(ON|OFF)")
_FLAGS_RE = re.compile(r"Flags:\s*OV=(\d)\s*UV=(\d)\s*OCD=(\d)\s*SCD=(\d)\s*XREADY=(\d)")


class BmsReader(Node):
    def __init__(self):
        super().__init__("bms_reader")

        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", BAUD_RATE)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        try:
            self._ser = serial.Serial(port, baud, timeout=0)  # non-blocking
            self.get_logger().info(f"BMS connected on {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().warn(f"Cannot open {port}: {e} — bms_reader idle, no data")
            self._ser = None

        self._rx = b""
        self._block_lines = []

        self._state_pub = self.create_publisher(BatteryState, "/battery/state", 10)
        self._raw_pub = self.create_publisher(String, "/battery/status_raw", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_timer(0.2, self._tick)

    def _tick(self):
        if self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            if n:
                self._rx += self._ser.read(n)
        except serial.SerialException as e:
            self.get_logger().warn(f"BMS serial read error: {e}", throttle_duration_sec=5.0)
            return

        while b"\n" in self._rx:
            raw, self._rx = self._rx.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if not line:
                if self._block_lines:
                    self._publish_block(self._block_lines)
                    self._block_lines = []
                continue
            # A new block's header line means the previous block (if we
            # never saw its Flags: line, e.g. after a dropped byte) is done.
            if line.startswith("---") and self._block_lines:
                self._publish_block(self._block_lines)
                self._block_lines = []
            self._block_lines.append(line)
            # Flags is always the last field in a block — flush as soon as
            # it arrives instead of waiting for a blank-line separator that
            # doesn't actually appear between blocks on this firmware.
            if line.startswith("Flags:"):
                self._publish_block(self._block_lines)
                self._block_lines = []

    def _publish_block(self, lines):
        text = " | ".join(lines)

        cells = []
        m = _CELLS_RE.search(text)
        if m:
            cells = [float(v) for v in m.group(1).split()]

        pack_v = None
        m = _PACK_RE.search(text)
        if m:
            pack_v = float(m.group(1))

        temp_c = None
        m = _TEMP_RE.search(text)
        if m:
            temp_c = float(m.group(1))

        curr_a = None
        m = _CURR_RE.search(text)
        if m:
            curr_a = float(m.group(1))

        chg_on = dsg_on = None
        m = _FETS_RE.search(text)
        if m:
            chg_on = m.group(1) == "ON"
            dsg_on = m.group(2) == "ON"

        flags = {}
        m = _FLAGS_RE.search(text)
        if m:
            flags = dict(zip(("OV", "UV", "OCD", "SCD", "XREADY"), (v == "1" for v in m.groups())))

        if not cells and pack_v is None:
            # Nothing recognizable in this block — don't publish a blank
            # BatteryState that would look like a real (zeroed-out) reading.
            self.get_logger().warn(f"Unparsed BMS block: {text!r}", throttle_duration_sec=5.0)
            return

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = pack_v if pack_v is not None else float("nan")
        state.current = curr_a if curr_a is not None else float("nan")
        state.temperature = temp_c if temp_c is not None else float("nan")
        state.cell_voltage = cells
        state.present = True
        if chg_on is None:
            state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif chg_on:
            state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif dsg_on:
            state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self._state_pub.publish(state)

        raw = String()
        raw.data = text
        self._raw_pub.publish(raw)

        self._publish_diagnostics(cells, pack_v, temp_c, curr_a, chg_on, dsg_on, flags)

    def _publish_diagnostics(self, cells, pack_v, temp_c, curr_a, chg_on, dsg_on, flags):
        fault = any(flags.values()) or dsg_on is False
        status = DiagnosticStatus()
        status.name = "bms: BQ76920"
        status.hardware_id = "bq76920"
        status.level = DiagnosticStatus.ERROR if fault else DiagnosticStatus.OK
        status.message = "Fault" if fault else "OK"

        def kv(key, value):
            status.values.append(KeyValue(key=key, value=str(value)))

        kv("Pack (V)", f"{pack_v:.2f}" if pack_v is not None else "?")
        for i, v in enumerate(cells, start=1):
            kv(f"Cell {i} (V)", f"{v:.3f}")
        kv("Temp (C)", f"{temp_c:.1f}" if temp_c is not None else "?")
        kv("Current (A)", f"{curr_a:.2f}" if curr_a is not None else "?")
        kv("CHG", "ON" if chg_on else "OFF" if chg_on is not None else "?")
        kv("DSG", "ON" if dsg_on else "OFF" if dsg_on is not None else "?")
        for name, tripped in flags.items():
            kv(f"Flag {name}", "TRIPPED" if tripped else "ok")

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(status)
        self._diag_pub.publish(arr)

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BmsReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
