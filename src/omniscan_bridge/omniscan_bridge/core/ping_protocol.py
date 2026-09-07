"""Cerulean / Blue Robotics **Ping Protocol** codec.

Pure Python, no ROS, no third-party dependencies. It is meant to be read and
understood, not treated as a black box, so the binary layout is written out
explicitly rather than hidden behind a code generator.

--------------------------------------------------------------------------
Frame layout
--------------------------------------------------------------------------

Every message on the wire, inbound or outbound, is::

    offset  size  field
    ------  ----  --------------------------------------------------------
    0       1     'B'  (0x42)          start byte 1
    1       1     'R'  (0x52)          start byte 2
    2       2     payload_length       uint16, little-endian
    4       2     message_id           uint16, little-endian
    6       1     src_device_id        uint8
    7       1     dst_device_id        uint8
    8       N     payload              N == payload_length
    8+N     2     checksum             uint16, little-endian

The checksum is the plain unsigned sum of **every byte from offset 0 up to but
not including the checksum itself**, truncated to 16 bits. It is a weak check —
it will not catch a transposition — which is why the parser also validates the
declared point count against the payload length (see ``decode_point_set``).

Total frame size is therefore ``8 + payload_length + 2``.

--------------------------------------------------------------------------
Payload layouts
--------------------------------------------------------------------------

.. warning::

   The frame layout above is the documented, stable Ping Protocol header. The
   **payload** layouts below are transcribed from the architecture brief, which
   names the fields but not, in every case, their order and width. They are
   written as explicit tables (``_POINT_SET_HEADER`` and friends) precisely so
   that reconciling them with Cerulean's real definition is a one-table edit and
   not an archaeology exercise.

   **Validate against Cerulean's published sample data before the first field
   deployment.** Tracked as Q8 in ``docs/open_questions.md``.

``OS3D_POINT_SET`` (3104) — the primary data message::

    uint32   ping_number
    float32  speed_of_sound       m/s, as actually used for this ping
    uint64   utc_msec             sonar's own clock, milliseconds since epoch
    uint32   num_points
    then num_points x 16 bytes:
        float32  angle            radians, in the SENSOR frame
        float32  tof              seconds, two-way time of flight
        float32  pwr              return power, arbitrary units
        uint8    pt_type          0 = no return, 1 = bottom detection
        uint8[3] padding          keeps each point 16-byte aligned

These are **not** XYZ coordinates. Converting them into a vessel-frame point is
our job and lives in :mod:`omniscan_bridge.core.geometry`.

``ATTITUDE_REPORT`` (504)::

    float32  pitch_deg
    float32  roll_deg

``END_PING_INFO`` (3010)::

    uint32   ping_number
    uint32   num_results
    float32  ping_duration_s

``OS3D_SET_PING_PARAMETERS`` (3024) — outbound::

    uint32   range_mm
    uint8    gain
    uint8[3] padding
    float32  ping_rate_hz

``SET_NTP_URL`` (18) — outbound; points the sonar at the Jetson's NTP server so
that ``utc_msec`` is comparable with our own clock::

    char[]   url, NUL-terminated
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# Frame constants
# --------------------------------------------------------------------------

START_BYTES = b"BR"
HEADER_SIZE = 8
CHECKSUM_SIZE = 2
#: Refuse to allocate for an absurd declared length; a corrupt length field must
#: not be able to make us buffer megabytes.
MAX_PAYLOAD_LENGTH = 65535

#: Default UDP/TCP port for the Omniscan 3D (brief, section 4).
DEFAULT_PORT = 62312

# Message IDs.
MSG_SET_NTP_URL = 18
MSG_ATTITUDE_REPORT = 504
MSG_END_PING_INFO = 3010
MSG_OS3D_SET_PING_PARAMETERS = 3024
MSG_OS3D_POINT_SET = 3104

MESSAGE_NAMES = {
    MSG_SET_NTP_URL: "SET_NTP_URL",
    MSG_ATTITUDE_REPORT: "ATTITUDE_REPORT",
    MSG_END_PING_INFO: "END_PING_INFO",
    MSG_OS3D_SET_PING_PARAMETERS: "OS3D_SET_PING_PARAMETERS",
    MSG_OS3D_POINT_SET: "OS3D_POINT_SET",
}

# Payload layouts, as struct format strings. '<' = little-endian, no padding.
_HEADER_FMT = struct.Struct("<2sHHBB")
_CHECKSUM_FMT = struct.Struct("<H")

#: ping_number, speed_of_sound, utc_msec, num_points
_POINT_SET_HEADER = struct.Struct("<IfQI")
#: angle, tof, pwr, pt_type, 3 bytes padding
_POINT = struct.Struct("<fffB3x")
POINT_SIZE = _POINT.size  # 16, asserted by a unit test

_ATTITUDE = struct.Struct("<ff")
_END_PING_INFO = struct.Struct("<IIf")
_SET_PING_PARAMETERS = struct.Struct("<IB3xf")

assert POINT_SIZE == 16, "point stride must be 16 bytes; see the layout above"


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


@dataclass
class PointSet:
    ping_number: int
    speed_of_sound: float
    utc_msec: int
    points: list[Point] = field(default_factory=list)
    #: True when ``num_points`` disagreed with the payload length. The points
    #: present are still returned — a truncated ping is more useful than none —
    #: but the caller must treat the ping as suspect.
    length_mismatch: bool = False


@dataclass
class AttitudeReport:
    pitch_deg: float
    roll_deg: float


@dataclass
class EndPingInfo:
    ping_number: int
    num_results: int
    ping_duration_s: float


# --------------------------------------------------------------------------
# Checksum
# --------------------------------------------------------------------------


def checksum(data: bytes) -> int:
    """Ping Protocol checksum: 16-bit truncated sum of the preceding bytes."""
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
    """Encode an ``OS3D_POINT_SET``. Used by the simulator and by the tests."""
    payload = bytearray(
        _POINT_SET_HEADER.pack(
            ps.ping_number, ps.speed_of_sound, ps.utc_msec, len(ps.points)
        )
    )
    for p in ps.points:
        payload += _POINT.pack(p.angle_rad, p.tof_s, p.power, p.pt_type)
    return encode_frame(MSG_OS3D_POINT_SET, bytes(payload), **frame_kwargs)


def encode_attitude_report(att: AttitudeReport, **frame_kwargs) -> bytes:
    return encode_frame(
        MSG_ATTITUDE_REPORT, _ATTITUDE.pack(att.pitch_deg, att.roll_deg), **frame_kwargs
    )


def encode_end_ping_info(info: EndPingInfo, **frame_kwargs) -> bytes:
    return encode_frame(
        MSG_END_PING_INFO,
        _END_PING_INFO.pack(info.ping_number, info.num_results, info.ping_duration_s),
        **frame_kwargs,
    )


def encode_set_ping_parameters(
    range_m: float, gain: int, ping_rate_hz: float, **frame_kwargs
) -> bytes:
    """Outbound command. Range travels as millimetres on the wire."""
    if not 0 <= gain <= 255:
        raise PingProtocolError(f"gain {gain} out of range 0..255")
    return encode_frame(
        MSG_OS3D_SET_PING_PARAMETERS,
        _SET_PING_PARAMETERS.pack(int(round(range_m * 1000.0)), gain, ping_rate_hz),
        **frame_kwargs,
    )


def encode_set_ntp_url(url: str, **frame_kwargs) -> bytes:
    """Point the sonar at our NTP server.

    Sent at startup. Everything in section 5 of the brief — recording the sonar
    stream and the trajectory separately and pairing them afterwards on
    ``utc_msec`` — depends on this having worked.
    """
    return encode_frame(MSG_SET_NTP_URL, url.encode("ascii") + b"\x00", **frame_kwargs)


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
            f"point set payload of {len(payload)} bytes is shorter than its header"
        )
    ping_number, sos, utc_msec, declared = _POINT_SET_HEADER.unpack_from(payload, 0)

    body = payload[_POINT_SET_HEADER.size :]
    available = len(body) // POINT_SIZE
    mismatch = available != declared
    count = min(declared, available)

    points = [
        Point(*_POINT.unpack_from(body, i * POINT_SIZE)) for i in range(count)
    ]
    return PointSet(
        ping_number=ping_number,
        speed_of_sound=sos,
        utc_msec=utc_msec,
        points=points,
        length_mismatch=mismatch,
    )


def decode_attitude_report(payload: bytes) -> AttitudeReport:
    if len(payload) < _ATTITUDE.size:
        raise PingProtocolError("attitude report payload too short")
    return AttitudeReport(*_ATTITUDE.unpack_from(payload, 0))


def decode_end_ping_info(payload: bytes) -> EndPingInfo:
    if len(payload) < _END_PING_INFO.size:
        raise PingProtocolError("end ping info payload too short")
    return EndPingInfo(*_END_PING_INFO.unpack_from(payload, 0))


#: Dispatch table used by :class:`PingParser` consumers.
PAYLOAD_DECODERS = {
    MSG_OS3D_POINT_SET: decode_point_set,
    MSG_ATTITUDE_REPORT: decode_attitude_report,
    MSG_END_PING_INFO: decode_end_ping_info,
}


def decode_payload(frame: Frame):
    """Decode a frame's payload, or return ``None`` for messages we ignore."""
    decoder = PAYLOAD_DECODERS.get(frame.message_id)
    return decoder(frame.payload) if decoder else None
