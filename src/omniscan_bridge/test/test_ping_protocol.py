"""The wire codec: framing, checksums, and every message we encode or decode."""

import math
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
    struct.pack_into("<H", payload, 8, 99)      # num_points is a u16 at offset 8

    decoded = pp.decode_point_set(bytes(payload))
    assert decoded.length_mismatch
    assert len(decoded.points) == 4, "the points that are there are still usable"


def test_the_point_set_header_is_eighty_bytes():
    """Transcribed from Cerulean's published OS3D_POINT_SET definition. If this
    changes, so has the layout, and the conversion downstream is wrong."""
    assert pp._POINT_SET_HEADER.size == 80
    frame = pp.encode_point_set(pp.PointSet(1, 1500.0, 0, [pp.Point(0.0, 0.01, 1.0, 1)] * 3))
    assert len(frame) == pp.HEADER_SIZE + 80 + 3 * 16 + pp.CHECKSUM_SIZE


def test_point_types_are_classifications_not_a_presence_flag():
    """0 is 'unclassified', not 'no return'; 2 is water column. Only bottom
    points belong in the bathymetry — a fish is not seabed."""
    ps = pp.PointSet(1, 1500.0, 0, [
        pp.Point(0.0, 0.01, 1.0, pp.PT_TYPE_UNCLASSIFIED),
        pp.Point(0.1, 0.02, 1.0, pp.PT_TYPE_BOTTOM),
        pp.Point(0.2, 0.03, 1.0, pp.PT_TYPE_WATER_COLUMN),
    ])
    decoded = pp.decode_point_set(pp.encode_point_set(ps)[8:-2])
    assert len(decoded.points) == 3
    assert [p.pt_type for p in decoded.bottom_points()] == [pp.PT_TYPE_BOTTOM]


def test_the_speed_of_sound_and_power_thresholds_survive_the_round_trip():
    """The thresholds are the device's own opinion of its data quality, which
    beats a fixed cutoff of ours."""
    ps = pp.PointSet(
        9, 1503.5, 1_700_000_000_000, [pp.Point(0.0, 0.01, 1.0, 1)],
        pwr_threshold_high=0.9, pwr_threshold_med=0.5, pwr_threshold_low=0.2,
        device_number=1, version=1,
    )
    decoded = pp.decode_point_set(pp.encode_point_set(ps)[8:-2])
    assert decoded.speed_of_sound == pytest.approx(1503.5, abs=1e-2)
    assert decoded.pwr_threshold_high == pytest.approx(0.9, abs=1e-6)
    assert decoded.pwr_threshold_low == pytest.approx(0.2, abs=1e-6)
    assert decoded.device_number == 1


def test_short_payload_raises_rather_than_guessing():
    with pytest.raises(pp.PingProtocolError):
        pp.decode_point_set(b"\x00\x01\x02")


def test_attitude_is_an_up_vector_and_angles_come_out_of_it():
    """The device does not send roll and pitch. It sends the world up vector in
    its own frame, and the angles are derived."""
    level = pp.decode_attitude_report(
        pp.encode_attitude_report(pp.AttitudeReport(0.0, 0.0, 1.0))[8:-2]
    )
    assert level.roll_deg == pytest.approx(0.0)
    assert level.pitch_deg == pytest.approx(0.0)

    # up_y = sin(10 deg) is a 10 degree roll, port up.
    rolled = pp.decode_attitude_report(
        pp.encode_attitude_report(
            pp.AttitudeReport(0.0, math.sin(math.radians(10)), math.cos(math.radians(10)))
        )[8:-2]
    )
    assert rolled.roll_deg == pytest.approx(10.0, abs=1e-4)
    assert rolled.pitch_deg == pytest.approx(0.0, abs=1e-4)

    # up_x = -sin(10 deg) is 10 degrees nose-up.
    pitched = pp.decode_attitude_report(
        pp.encode_attitude_report(
            pp.AttitudeReport(-math.sin(math.radians(10)), 0.0, math.cos(math.radians(10)))
        )[8:-2]
    )
    assert pitched.pitch_deg == pytest.approx(10.0, abs=1e-4)
    assert pitched.roll_deg == pytest.approx(0.0, abs=1e-4)


def test_end_ping_info_carries_the_rate_the_device_actually_achieved():
    """Better than timing arrivals here, which also measures the network."""
    info = pp.decode_end_ping_info(
        pp.encode_end_ping_info(
            pp.EndPingInfo(
                ping_number=7, ping_hz_realized=4.2, range_start_m=0.0,
                range_end_m=27.5, gain_index=3, utc_msec=1_700_000_000_000,
                n_range_bins=400, samples_per_range_bin=4,
            )
        )[8:-2]
    )
    assert info.ping_number == 7
    assert info.ping_hz_realized == pytest.approx(4.2, abs=1e-5)
    assert info.range_end_m == pytest.approx(27.5)
    assert info.gain_index == 3
    assert info.utc_msec == 1_700_000_000_000
    assert not info.has_water_temperature      # -1000 means "no sensor"


def test_end_ping_info_is_eighty_bytes():
    assert pp._END_PING_INFO.size == 80


def test_ping_parameters_carry_a_period_not_a_rate():
    """The wire field is msec_per_ping; pings per second = 1000 / it."""
    params = pp.PingParameters.from_rate(5.0, end_m=30.0)
    assert params.msec_per_ping == 200
    assert params.ping_rate_hz == pytest.approx(5.0)

    decoded = pp.decode_set_ping_parameters(
        pp.encode_set_ping_parameters(params)[8:-2]
    )
    assert decoded.msec_per_ping == 200
    assert decoded.end_m == pytest.approx(30.0)


def test_atof_data_is_enabled_by_default():
    """Without it the sonar produces no angle/time-of-flight points at all,
    which is the entire output this bridge consumes."""
    decoded = pp.decode_set_ping_parameters(
        pp.encode_set_ping_parameters(pp.PingParameters())[8:-2]
    )
    assert decoded.enable_atof_data is True


def test_pinging_is_started_and_stopped_with_a_flag_not_a_zero_rate():
    stopped = pp.decode_set_ping_parameters(
        pp.encode_set_ping_parameters(pp.PingParameters(ping_enable=False))[8:-2]
    )
    assert stopped.ping_enable is False
    assert stopped.msec_per_ping > 0, "the rate is not how you stop it"


def test_auto_gain_is_minus_one_and_out_of_range_gain_is_rejected():
    assert pp.PingParameters().gain_index == -1        # Cerulean's recommendation
    with pytest.raises(pp.PingProtocolError):
        pp.encode_set_ping_parameters(pp.PingParameters(gain_index=11))
    with pytest.raises(pp.PingProtocolError):
        pp.encode_set_ping_parameters(pp.PingParameters(gain_index=-2))


def test_range_steps_outside_the_documented_bounds_are_rejected():
    with pytest.raises(pp.PingProtocolError):
        pp.encode_set_ping_parameters(pp.PingParameters(n_range_steps=100))
    with pytest.raises(pp.PingProtocolError):
        pp.encode_set_ping_parameters(pp.PingParameters(n_range_steps=1001))


def test_nmea_wrapper_carries_a_sentence_and_terminates_it():
    """This is the only way position and heading get into a Cerulean log."""
    frame = pp.encode_nmea_wrapper("$GPGGA,123519,4807.038,N")
    payload = frame[pp.HEADER_SIZE : -pp.CHECKSUM_SIZE]
    assert payload.endswith(b"\r\n")
    assert pp.decode_nmea_wrapper(payload) == "$GPGGA,123519,4807.038,N"


def test_unknown_message_ids_decode_to_none_rather_than_raising():
    """The device sends more than we consume. That is not an error."""
    assert pp.decode_payload(pp.Frame(9999, 0, 0, b"\x00" * 8)) is None
