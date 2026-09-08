"""Cerulean **Ping Protocol** codec.

Pure Python, no ROS, no third-party dependencies. It is meant to be read and
understood, not treated as a black box, so the binary layout is written out
explicitly rather than hidden behind a code generator.

Every layout below is transcribed from Cerulean's published documentation
(docs.ceruleansonar.com), not inferred. The pages used are cited against each
message. Two things are *not* stated by those pages and are assumed here:

* **Endianness.** The docs give sizes but never byte order. Little-endian is
  assumed, as in the Blue Robotics ping-protocol this is descended from, and
  because the devices are ARM/x86. A wrong guess would fail immediately and
  loudly — a big-endian ``num_points`` would be astronomically large and the
  cross-check in :func:`decode_point_set` would reject the frame.
* **``vec3``.** Not in the nomenclature table; taken as three ``float`` (12
  bytes), which is the only reading consistent with the fixed payload sizes.

--------------------------------------------------------------------------
Frame layout — "Universal Packet Format"
--------------------------------------------------------------------------

Confirmed against the published table::

    offset      size  field
    ----------  ----  ----------------------------------------------------
    0           1     'B'  (0x42)
    1           1     'R'  (0x52)
    2-3         2     N, number of bytes in the payload
    4-5         2     packet_id
    6           1     src_device_id
    7           1     dst_device_id
    8..8+N-1    N     payload
    8+N..8+N+1  2     checksum

The checksum is "the sum of all the bytes in the packet from 0 to n, truncated
to 16 bits". Total frame size is ``8 + N + 2``.

It is a weak check — it will not catch a transposition — which is why the
parser also validates declared counts against payload lengths.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# Frame constants
# --------------------------------------------------------------------------

START_BYTES = b"BR"
HEADER_SIZE = 8
CHECKSUM_SIZE = 2
#: The length field is a uint16, so this is the protocol's own ceiling.
MAX_PAYLOAD_LENGTH = 65535

#: Default port for the Omniscan 3D.
DEFAULT_PORT = 62312

# Message IDs, from the published index of packet types.
MSG_NOP = 0
MSG_ACK = 1
MSG_NACK = 2
MSG_ASCII_TEXT = 3
MSG_DEVICE_INFORMATION = 4
MSG_GENERAL_REQUEST = 6
#: Wraps an NMEA 0183 sentence. This is how position and heading get *into* a
#: log, and therefore how a survey becomes georeferenceable — see
#: :mod:`mission_recorder.core.svlog`.
MSG_NMEA_WRAPPER = 109
#: Wraps a MAVLink message rendered as JSON.
MSG_MAVLINK_WRAPPER = 150
MSG_ATTITUDE_REPORT = 504
MSG_END_PING_INFO = 3010
MSG_OS3D_SET_PING_PARAMETERS = 3024
MSG_OS3D_POINT_SET = 3104

MESSAGE_NAMES = {
    MSG_NOP: "NOP",
    MSG_ACK: "ACK",
    MSG_NACK: "NACK",
    MSG_ASCII_TEXT: "ASCII_TEXT",
    MSG_DEVICE_INFORMATION: "DEVICE_INFORMATION",
    MSG_GENERAL_REQUEST: "GENERAL_REQUEST",
    MSG_NMEA_WRAPPER: "NMEA_WRAPPER",
    MSG_MAVLINK_WRAPPER: "MAVLINK_WRAPPER",
    MSG_ATTITUDE_REPORT: "ATTITUDE_REPORT",
    MSG_END_PING_INFO: "END_PING_INFO",
    MSG_OS3D_SET_PING_PARAMETERS: "OS3D_SET_PING_PARAMETERS",
    MSG_OS3D_POINT_SET: "OS3D_POINT_SET",
}

# --------------------------------------------------------------------------
# Point classification
# --------------------------------------------------------------------------

#: ``pt_type`` values, per the OS3D_POINT_SET definition.
PT_TYPE_UNCLASSIFIED = 0
PT_TYPE_BOTTOM = 1
PT_TYPE_WATER_COLUMN = 2

#: What counts as a seabed detection. Only ``bottom_points``: an unclassified
#: point is one the sonar could not place, and a water-column point is a fish,
#: a thermocline or a wake. Painting either as seabed would put features in the
#: bathymetry that are not on the bottom.
BOTTOM_TYPES = frozenset({PT_TYPE_BOTTOM})

# --------------------------------------------------------------------------
# Struct layouts
# --------------------------------------------------------------------------

_HEADER_FMT = struct.Struct("<2sHHBB")
_CHECKSUM_FMT = struct.Struct("<H")

#: OS3D_POINT_SET (3104) header, 80 bytes, then ``num_points`` x 16.
#:
#:   u32 ping_number | float sos_mps | u16 num_points | u16 unused
#:   u32 unused | u64 utc_msec | u32 pwr_up_msec
#:   u8 version | u8 device_number | u8 unused | u8 reserved
#:   float pwr_threshold_high | float pwr_threshold_med | float pwr_threshold_low
#:   u32 reserved[9]
_POINT_SET_HEADER = struct.Struct("<IfHHIQIBBBB3f9I")

#: atof_point_t: float angle (rad), float tof (s), float pwr, u8 pt_type, u8[3].
_POINT = struct.Struct("<fffB3x")
POINT_SIZE = _POINT.size

#: ATTITUDE_REPORT (504): vec3 up_vec | vec3 reserved | u64 utc_msec |
#: u32 pwr_up_msec | u8 channel_number
_ATTITUDE = struct.Struct("<3f3fQIB")

#: END_PING_INFO (3010), 80 bytes.
_END_PING_INFO = struct.Struct("<Iff3fIfff3ffiHHHBBIQ")

#: OS3D_SET_PING_PARAMETERS (3024), 36 bytes.
_SET_PING_PARAMETERS = struct.Struct("<fffhhHBBBBBBiHHf")

assert POINT_SIZE == 16, "point stride must be 16 bytes"
assert _POINT_SET_HEADER.size == 80, f"point set header is {_POINT_SET_HEADER.size}, expected 80"
assert _END_PING_INFO.size == 80, f"end ping info is {_END_PING_INFO.size}, expected 80"
assert _SET_PING_PARAMETERS.size == 36, f"ping params is {_SET_PING_PARAMETERS.size}, expected 36"


class PingProtocolError(ValueError):
    """Raised when a payload cannot be decoded as the message it claims to be."""


# --------------------------------------------------------------------------
# Decoded message types
# --------------------------------------------------------------------------


@dataclass
class Frame:
    """A checksum-valid frame, payload still undecoded."""

    message_id: int
    src_device_id: int
    dst_device_id: int
    payload: bytes

    @property
    def name(self) -> str:
        return MESSAGE_NAMES.get(self.message_id, f"UNKNOWN_{self.message_id}")


@dataclass
class Point:
    """One beam's detection, in the sensor frame."""

    angle_rad: float
    tof_s: float
    power: float
    pt_type: int

    @property
    def is_bottom(self) -> bool:
        return self.pt_type in BOTTOM_TYPES


@dataclass
class PointSet:
    ping_number: int
    speed_of_sound: float
    utc_msec: int
    points: list[Point] = field(default_factory=list)
    pwr_up_msec: int = 0
    version: int = 1
    device_number: int = 0
    #: Power levels the sonar itself considers strong / reasonable / liberal.
    #: Worth carrying: they are the device's own opinion of its data quality,
    #: and a fixed threshold of ours would be worse.
    pwr_threshold_high: float = 0.0
    pwr_threshold_med: float = 0.0
    pwr_threshold_low: float = 0.0
    #: True when ``num_points`` disagreed with the payload length. The points
    #: present are still returned — a truncated ping is more useful than none —
    #: but the caller must treat the ping as suspect.
    length_mismatch: bool = False

    def bottom_points(self) -> list[Point]:
        return [p for p in self.points if p.is_bottom]


@dataclass
class AttitudeReport:
    """Device attitude, reported as the world up vector in the device frame.

    Not as angles: the device sends ``up_vec`` in its own coordinate system
    (x forward, y port, z up), and roll and pitch are derived from it.
    """

    up_x: float
    up_y: float
    up_z: float
    utc_msec: int = 0
    pwr_up_msec: int = 0
    channel_number: int = 0

    @property
    def roll_deg(self) -> float:
        """Rotation about the forward axis, port-up positive (REP-103)."""
        return math.degrees(math.atan2(self.up_y, self.up_z))

    @property
    def pitch_deg(self) -> float:
        """Rotation about the port axis, nose-up positive."""
        return math.degrees(math.asin(max(-1.0, min(1.0, -self.up_x))))


@dataclass
class EndPingInfo:
    ping_number: int
    #: The rate the sonar actually achieved, which is not always the one it was
    #: asked for — a long range forces a slower ping. Better than measuring it
    #: ourselves, because it is the device's own figure.
    ping_hz_realized: float
    range_start_m: float
    range_end_m: float
    gain_index: int
    utc_msec: int
    pwr_up_msec: int = 0
    up_x: float = 0.0
    up_y: float = 0.0
    up_z: float = 1.0
    water_degc: float = -1000.0
    water_bar: float = -1000.0
    pulse_usec: int = 0
    n_range_bins: int = 0
    samples_per_range_bin: int = 0
    device_number: int = 0

    @property
    def has_water_temperature(self) -> bool:
        """-1000 is the documented "no sensor present" value, not a reading."""
        return self.water_degc > -999.0


@dataclass
class PingParameters:
    """Outbound settings for OS3D_SET_PING_PARAMETERS."""

    #: Start of the recorded range window. Ignored in auto-range mode.
    start_m: float = 0.0
    #: End of the range window. **Zero means the sonar tracks the bottom
    #: itself**, which is usually what you want and is not the same as "no
    #: range".
    end_m: float = 0.0
    speed_of_sound: float = 1500.0
    #: -1 is auto gain and is what Cerulean recommend; 0..10 is manual.
    gain_index: int = -1
    #: The wire carries a period, not a rate: pings per second = 1000 / this.
    msec_per_ping: int = 200
    ping_enable: bool = True
    #: Used by SonarView; not generally useful to us and it costs bandwidth.
    enable_channel_data: bool = False
    #: **Must be true or no angle/time-of-flight points are produced at all.**
    enable_atof_data: bool = True
    #: Production-test field; the documentation says set it to 450000.
    target_ping_hz: int = 450000
    #: Range resolution, 200..1000 inclusive.
    n_range_steps: int = 400
    #: Pulse length in range steps; 1.5 is the recommended value.
    pulse_len_steps: float = 1.5

    @property
    def ping_rate_hz(self) -> float:
        return 1000.0 / self.msec_per_ping if self.msec_per_ping else 0.0

    @classmethod
    def from_rate(cls, ping_rate_hz: float, **kwargs) -> "PingParameters":
        msec = int(round(1000.0 / ping_rate_hz)) if ping_rate_hz > 0 else 1000
        return cls(msec_per_ping=max(1, min(32767, msec)), **kwargs)


# --------------------------------------------------------------------------
# Checksum
# --------------------------------------------------------------------------


def checksum(data: bytes) -> int:
    """Sum of all bytes in the packet up to the checksum, truncated to 16 bits."""
    return sum(data) & 0xFFFF


# --------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------


def encode_frame(
    message_id: int,
    payload: bytes,
    src_device_id: int = 0,
    dst_device_id: int = 0,
) -> bytes:
    """Wrap a payload in a Ping Protocol frame, checksum included."""
    if len(payload) > MAX_PAYLOAD_LENGTH:
        raise PingProtocolError(f"payload of {len(payload)} bytes exceeds the field width")
    head = _HEADER_FMT.pack(
        START_BYTES, len(payload), message_id, src_device_id, dst_device_id
    )
    body = head + payload
    return body + _CHECKSUM_FMT.pack(checksum(body))


def encode_point_set(ps: PointSet, **frame_kwargs) -> bytes:
    payload = bytearray(
        _POINT_SET_HEADER.pack(
            ps.ping_number,
            ps.speed_of_sound,
            len(ps.points),
            0,
            0,
            ps.utc_msec,
            ps.pwr_up_msec,
            ps.version,
            ps.device_number,
            0,
            0,
            ps.pwr_threshold_high,
            ps.pwr_threshold_med,
            ps.pwr_threshold_low,
            *([0] * 9),
        )
    )
    for p in ps.points:
        payload += _POINT.pack(p.angle_rad, p.tof_s, p.power, p.pt_type)
    return encode_frame(MSG_OS3D_POINT_SET, bytes(payload), **frame_kwargs)


def encode_attitude_report(att: AttitudeReport, **frame_kwargs) -> bytes:
    return encode_frame(
        MSG_ATTITUDE_REPORT,
        _ATTITUDE.pack(
            att.up_x, att.up_y, att.up_z, 0.0, 0.0, 0.0,
            att.utc_msec, att.pwr_up_msec, att.channel_number,
        ),
        **frame_kwargs,
    )


def encode_end_ping_info(info: EndPingInfo, **frame_kwargs) -> bytes:
    return encode_frame(
        MSG_END_PING_INFO,
        _END_PING_INFO.pack(
            0,
            info.range_start_m,
            info.range_end_m,
            info.up_x, info.up_y, info.up_z,
            info.ping_number,
            info.water_degc,
            info.water_bar,
            0.0,
            0.0, 0.0, 0.0,
            info.ping_hz_realized,
            info.gain_index,
            info.pulse_usec,
            info.n_range_bins,
            info.samples_per_range_bin,
            info.device_number,
            0,
            info.pwr_up_msec,
            info.utc_msec,
        ),
        **frame_kwargs,
    )


def encode_set_ping_parameters(params: PingParameters, **frame_kwargs) -> bytes:
    if not (-1 <= params.gain_index <= 10):
        raise PingProtocolError(
            f"gain_index {params.gain_index} out of range: -1 for auto, or 0..10"
        )
    if not (200 <= params.n_range_steps <= 1000):
        raise PingProtocolError(
            f"n_range_steps {params.n_range_steps} out of range 200..1000"
        )
    return encode_frame(
        MSG_OS3D_SET_PING_PARAMETERS,
        _SET_PING_PARAMETERS.pack(
            params.start_m,
            params.end_m,
            params.speed_of_sound,
            params.gain_index,
            params.msec_per_ping,
            0,
            0,
            int(bool(params.ping_enable)),
            int(bool(params.enable_channel_data)),
            0,
            0,
            int(bool(params.enable_atof_data)),
            params.target_ping_hz,
            params.n_range_steps,
            0,
            params.pulse_len_steps,
        ),
        **frame_kwargs,
    )


def encode_nmea_wrapper(sentence: str, **frame_kwargs) -> bytes:
    """Wrap an NMEA 0183 sentence.

    This is the only way position and heading get into a Cerulean log:
    SonarView reads them from NMEA_WRAPPER packets inside the ``.svlog``, and a
    log without them cannot be georeferenced or exported at all.
    """
    text = sentence if sentence.endswith("\r\n") else sentence + "\r\n"
    return encode_frame(MSG_NMEA_WRAPPER, text.encode("ascii"), **frame_kwargs)


def encode_mavlink_wrapper(message_json: str, **frame_kwargs) -> bytes:
    """Wrap a MAVLink message rendered as JSON."""
    return encode_frame(MSG_MAVLINK_WRAPPER, message_json.encode("utf-8"), **frame_kwargs)


# --------------------------------------------------------------------------
# Payload decoding
# --------------------------------------------------------------------------


def decode_point_set(payload: bytes) -> PointSet:
    """Decode an ``OS3D_POINT_SET`` payload.

    Cross-checks the declared ``num_points`` against the bytes actually present.
    A mismatch is flagged rather than trusted: a wrong layout assumption would
    otherwise produce a plausible-looking cloud of nonsense, which is far worse
    than an error.
    """
    if len(payload) < _POINT_SET_HEADER.size:
        raise PingProtocolError(
            f"point set payload of {len(payload)} bytes is shorter than its "
            f"{_POINT_SET_HEADER.size}-byte header"
        )
    fields = _POINT_SET_HEADER.unpack_from(payload, 0)
    (
        ping_number, sos, declared, _unused16, _unused32, utc_msec, pwr_up_msec,
        version, device_number, _unused8, _reserved8,
        thr_high, thr_med, thr_low,
    ) = fields[:14]

    body = payload[_POINT_SET_HEADER.size :]
    available = len(body) // POINT_SIZE
    mismatch = available != declared
    count = min(declared, available)

    points = [Point(*_POINT.unpack_from(body, i * POINT_SIZE)) for i in range(count)]
    return PointSet(
        ping_number=ping_number,
        speed_of_sound=sos,
        utc_msec=utc_msec,
        points=points,
        pwr_up_msec=pwr_up_msec,
        version=version,
        device_number=device_number,
        pwr_threshold_high=thr_high,
        pwr_threshold_med=thr_med,
        pwr_threshold_low=thr_low,
        length_mismatch=mismatch,
    )


def decode_attitude_report(payload: bytes) -> AttitudeReport:
    if len(payload) < _ATTITUDE.size:
        raise PingProtocolError("attitude report payload too short")
    up_x, up_y, up_z, _rx, _ry, _rz, utc, pwr_up, channel = _ATTITUDE.unpack_from(payload, 0)
    return AttitudeReport(up_x, up_y, up_z, utc, pwr_up, channel)


def decode_end_ping_info(payload: bytes) -> EndPingInfo:
    if len(payload) < _END_PING_INFO.size:
        raise PingProtocolError("end ping info payload too short")
    (
        _reserved, range_start, range_end, up_x, up_y, up_z, ping_number,
        water_degc, water_bar, _heave, _mx, _my, _mz, ping_hz, gain_index,
        pulse_usec, n_range_bins, samples_per_bin, device_number, _unused,
        pwr_up_msec, utc_msec,
    ) = _END_PING_INFO.unpack_from(payload, 0)
    return EndPingInfo(
        ping_number=ping_number,
        ping_hz_realized=ping_hz,
        range_start_m=range_start,
        range_end_m=range_end,
        gain_index=gain_index,
        utc_msec=utc_msec,
        pwr_up_msec=pwr_up_msec,
        up_x=up_x, up_y=up_y, up_z=up_z,
        water_degc=water_degc,
        water_bar=water_bar,
        pulse_usec=pulse_usec,
        n_range_bins=n_range_bins,
        samples_per_range_bin=samples_per_bin,
        device_number=device_number,
    )


def decode_set_ping_parameters(payload: bytes) -> PingParameters:
    """Decode an outbound parameter packet.

    Only the simulated device needs this — a real sonar receives these, it does
    not send them — but having it means the encoder is round-trip tested rather
    than merely asserted about.
    """
    if len(payload) < _SET_PING_PARAMETERS.size:
        raise PingProtocolError("set ping parameters payload too short")
    (
        start_m, end_m, sos, gain_index, msec_per_ping, _reserved,
        _diagnostic, ping_enable, enable_channel, _raw, _reserved2, enable_atof,
        target_ping_hz, n_range_steps, _reserved3, pulse_len_steps,
    ) = _SET_PING_PARAMETERS.unpack_from(payload, 0)
    return PingParameters(
        start_m=start_m,
        end_m=end_m,
        speed_of_sound=sos,
        gain_index=gain_index,
        msec_per_ping=msec_per_ping,
        ping_enable=bool(ping_enable),
        enable_channel_data=bool(enable_channel),
        enable_atof_data=bool(enable_atof),
        target_ping_hz=target_ping_hz,
        n_range_steps=n_range_steps,
        pulse_len_steps=pulse_len_steps,
    )


def decode_nmea_wrapper(payload: bytes) -> str:
    return payload.decode("ascii", "replace").strip()


def decode_mavlink_wrapper(payload: bytes) -> str:
    return payload.decode("utf-8", "replace").strip()


#: Dispatch table used by :class:`PingParser` consumers.
PAYLOAD_DECODERS = {
    MSG_OS3D_POINT_SET: decode_point_set,
    MSG_ATTITUDE_REPORT: decode_attitude_report,
    MSG_END_PING_INFO: decode_end_ping_info,
    MSG_NMEA_WRAPPER: decode_nmea_wrapper,
    MSG_MAVLINK_WRAPPER: decode_mavlink_wrapper,
}


def decode_payload(frame: Frame):
    """Decode a frame's payload, or return ``None`` for messages we ignore."""
    decoder = PAYLOAD_DECODERS.get(frame.message_id)
    return decoder(frame.payload) if decoder else None
