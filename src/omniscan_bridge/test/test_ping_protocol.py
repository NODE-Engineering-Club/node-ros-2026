"""The wire codec: framing, checksums, and every message we encode or decode."""

import struct

import pytest
from omniscan_bridge.core import ping_protocol as pp


def test_a_point_is_sixteen_bytes():
    """The stride the whole point array depends on. If this changes, the layout
    assumption in the module docstring has changed with it."""
    assert pp.POINT_SIZE == 16


def test_frame_layout_is_what_the_docstring_says():
    frame = pp.encode_frame(pp.MSG_ATTITUDE_REPORT, b"\x01\x02\x03\x04")
    assert frame[:2] == b"BR"
    assert struct.unpack_from("<H", frame, 2)[0] == 4          # payload length
    assert struct.unpack_from("<H", frame, 4)[0] == 504        # message id
    assert len(frame) == pp.HEADER_SIZE + 4 + pp.CHECKSUM_SIZE


def test_checksum_covers_everything_before_itself():
    frame = pp.encode_frame(pp.MSG_ATTITUDE_REPORT, b"\xde\xad\xbe\xef")
    body, declared = frame[:-2], struct.unpack_from("<H", frame, len(frame) - 2)[0]
    assert declared == pp.checksum(body)
    assert declared == sum(body) & 0xFFFF


def test_point_set_round_trip():
    original = pp.PointSet(
        ping_number=42,
        speed_of_sound=1503.5,
        utc_msec=1_700_000_000_123,
        points=[
            pp.Point(-0.5, 0.021, 0.9, 1),
            pp.Point(0.0, 0.0, 0.0, 0),
            pp.Point(0.5, 0.03, 0.4, 1),
        ],
    )
    frame = pp.encode_point_set(original)
    decoded = pp.decode_point_set(frame[pp.HEADER_SIZE : -pp.CHECKSUM_SIZE])

    assert decoded.ping_number == 42
    assert decoded.utc_msec == 1_700_000_000_123
    assert decoded.speed_of_sound == pytest.approx(1503.5, abs=1e-2)
    assert not decoded.length_mismatch
    assert len(decoded.points) == 3
    assert decoded.points[0].angle_rad == pytest.approx(-0.5, abs=1e-6)
    assert decoded.points[1].pt_type == 0


def test_declared_count_disagreeing_with_the_payload_is_flagged_not_trusted():
    """A wrong layout assumption must fail loudly. Silently producing a
    plausible cloud of nonsense is far worse than an error."""
    ps = pp.PointSet(1, 1500.0, 0, [pp.Point(0.0, 0.01, 1.0, 1)] * 4)
    payload = bytearray(pp.encode_point_set(ps)[pp.HEADER_SIZE : -pp.CHECKSUM_SIZE])
    struct.pack_into("<I", payload, 16, 99)     # claim 99 points, send 4

    decoded = pp.decode_point_set(bytes(payload))
    assert decoded.length_mismatch
    assert len(decoded.points) == 4, "the points that are there are still usable"


def test_short_payload_raises_rather_than_guessing():
    with pytest.raises(pp.PingProtocolError):
        pp.decode_point_set(b"\x00\x01\x02")


def test_attitude_and_end_ping_info_round_trip():
    att = pp.decode_attitude_report(
        pp.encode_attitude_report(pp.AttitudeReport(-3.25, 11.5))[8:-2]
    )
    assert att.pitch_deg == pytest.approx(-3.25)
    assert att.roll_deg == pytest.approx(11.5)

    info = pp.decode_end_ping_info(
        pp.encode_end_ping_info(pp.EndPingInfo(7, 201, 0.2))[8:-2]
    )
    assert (info.ping_number, info.num_results) == (7, 201)
    assert info.ping_duration_s == pytest.approx(0.2)


def test_set_ping_parameters_sends_range_as_millimetres():
    frame = pp.encode_set_ping_parameters(range_m=30.0, gain=4, ping_rate_hz=5.0)
    range_mm, gain, rate = struct.unpack_from("<IB3xf", frame, pp.HEADER_SIZE)
    assert range_mm == 30_000
    assert gain == 4
    assert rate == pytest.approx(5.0)


def test_gain_outside_a_byte_is_rejected():
    with pytest.raises(pp.PingProtocolError):
        pp.encode_set_ping_parameters(30.0, 999, 5.0)


def test_ntp_url_is_nul_terminated():
    """Section 5 depends entirely on this having worked."""
    frame = pp.encode_set_ntp_url("192.168.2.1")
    payload = frame[pp.HEADER_SIZE : -pp.CHECKSUM_SIZE]
    assert payload == b"192.168.2.1\x00"


def test_unknown_message_ids_decode_to_none_rather_than_raising():
    """The device sends more than we consume. That is not an error."""
    assert pp.decode_payload(pp.Frame(9999, 0, 0, b"\x00" * 8)) is None
