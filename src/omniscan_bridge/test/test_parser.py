"""Parser robustness: partial frames, corruption, truncation, reordering.

These are the cases a real link produces and a bench test never does.
"""

import pytest
from omniscan_bridge.core import ping_protocol as pp
from omniscan_bridge.core.parser import PingParser


def point_set(ping_number: int, n_points: int = 8, utc: int = 0) -> bytes:
    return pp.encode_point_set(
        pp.PointSet(
            ping_number=ping_number,
            speed_of_sound=1500.0,
            utc_msec=utc or 1_700_000_000_000 + ping_number * 200,
            points=[pp.Point(i * 0.01, 0.02 + i * 1e-4, 0.5, 1) for i in range(n_points)],
        )
    )


def test_parses_a_clean_stream():
    parser = PingParser()
    stream = b"".join(point_set(i) for i in range(1, 6))
    frames = parser.feed(stream)
    assert len(frames) == 5
    assert all(f.message_id == pp.MSG_OS3D_POINT_SET for f in frames)
    assert parser.stats.bytes_discarded == 0
    assert parser.stats.checksum_errors == 0


def test_a_frame_split_across_reads_is_reassembled():
    parser = PingParser()
    stream = point_set(1)
    for i in range(0, len(stream) - 1):
        assert parser.feed(stream[i : i + 1]) == []
    frames = parser.feed(stream[-1:])
    assert len(frames) == 1


def test_a_start_marker_split_across_reads_survives():
    """'B' at the end of one read and 'R' at the start of the next."""
    parser = PingParser()
    stream = point_set(1)
    assert parser.feed(stream[:1]) == []      # just the 'B'
    frames = parser.feed(stream[1:])
    assert len(frames) == 1


def test_leading_rubbish_is_discarded_and_counted():
    parser = PingParser()
    frames = parser.feed(b"\x00\xff garbage " + point_set(1))
    assert len(frames) == 1
    assert parser.stats.bytes_discarded == len(b"\x00\xff garbage ")


def test_a_corrupt_frame_is_rejected_and_the_next_one_still_parses():
    """The single most important property: one bad frame must not desynchronise
    the stream for good."""
    parser = PingParser()
    good, bad = point_set(1), bytearray(point_set(2))
    bad[20] ^= 0xFF                                  # corrupt a payload byte
    frames = parser.feed(good + bytes(bad) + point_set(3))

    assert parser.stats.checksum_errors == 1
    assert len(frames) == 2
    decoded = [parser.decode(f) for f in frames]
    assert [d.ping_number for d in decoded] == [1, 3]


def test_bytes_lost_from_the_middle_of_the_stream_are_survived():
    parser = PingParser()
    stream = bytearray(b"".join(point_set(i) for i in range(1, 6)))
    del stream[200:290]                              # a dropped segment
    frames = parser.feed(bytes(stream))

    assert len(frames) >= 3, "must resynchronise, not give up"
    assert parser.stats.bytes_discarded > 0, "the loss must be visible, not silent"


def test_an_absurd_length_field_does_not_wedge_the_parser():
    """A corrupt length must not make us wait forever for bytes that never come."""
    parser = PingParser()
    bogus = bytearray(point_set(1))
    bogus[2:4] = (65000).to_bytes(2, "little")       # claim a huge payload
    frames = parser.feed(bytes(bogus) + point_set(2))
    assert len(frames) == 1
    assert parser.decode(frames[0]).ping_number == 2


def test_the_buffer_is_bounded():
    parser = PingParser()
    parser.feed(b"\x00" * (PingParser.MAX_BUFFER + 10_000))
    assert len(parser._buf) <= PingParser.MAX_BUFFER


# -- ping accounting -----------------------------------------------------


def decode_all(parser: PingParser, stream: bytes):
    return [parser.decode(f) for f in parser.feed(stream)]


def test_no_loss_reported_on_a_clean_sequence():
    parser = PingParser()
    decode_all(parser, b"".join(point_set(i) for i in range(1, 21)))
    assert parser.stats.pings_seen == 20
    assert parser.stats.pings_missing == 0
    assert parser.stats.packet_loss_ratio == 0.0


def test_a_gap_in_ping_numbers_is_measured_as_loss():
    """The sonar increments ping_number whether or not the packet reaches us, so
    a gap is evidence, not a guess."""
    parser = PingParser()
    decode_all(parser, point_set(1) + point_set(2) + point_set(6) + point_set(7))
    assert parser.stats.pings_missing == 3           # 3, 4, 5
    assert parser.stats.pings_seen == 4
    assert parser.stats.packet_loss_ratio == pytest.approx(3 / 7)


def test_a_late_ping_is_recognised_as_late_not_as_new():
    parser = PingParser()
    decode_all(parser, point_set(1) + point_set(3) + point_set(2) + point_set(4))
    assert parser.stats.pings_out_of_order == 1
    assert parser.stats.pings_missing == 0, "the gap was filled, not still missing"


def test_a_duplicate_ping_is_not_double_counted():
    parser = PingParser()
    decode_all(parser, point_set(1) + point_set(2) + point_set(2))
    assert parser.stats.pings_seen == 2
    assert parser.stats.pings_out_of_order == 1


def test_a_sonar_restart_is_not_reported_as_massive_packet_loss():
    parser = PingParser()
    decode_all(parser, b"".join(point_set(i) for i in range(5000, 5005)))
    decode_all(parser, point_set(1) + point_set(2))
    assert parser.stats.pings_missing == 0
    assert parser.stats.packet_loss_ratio == 0.0


def test_reset_clears_stream_state_but_keeps_the_tally():
    parser = PingParser()
    decode_all(parser, point_set(1) + point_set(2))
    parser.feed(b"half a frame \x42\x52")
    parser.reset()
    assert parser._buf == b""
    assert parser.stats.pings_seen == 2              # cumulative counters survive


def test_length_mismatches_are_counted_separately_from_corruption():
    """A persistent mismatch means our layout assumption is wrong (Q8), not that
    the sonar is faulty. Conflating the two would send someone hunting a cable."""
    import struct

    parser = PingParser()
    ps = pp.PointSet(1, 1500.0, 0, [pp.Point(0.0, 0.01, 1.0, 1)] * 4)
    payload = bytearray(pp.encode_point_set(ps)[pp.HEADER_SIZE : -pp.CHECKSUM_SIZE])
    struct.pack_into("<I", payload, 16, 99)
    frame = pp.encode_frame(pp.MSG_OS3D_POINT_SET, bytes(payload))

    decode_all(parser, frame)
    assert parser.stats.length_mismatches == 1
    assert parser.stats.checksum_errors == 0


def test_a_plausible_but_wrong_length_stalls_only_briefly():
    """A corrupt length inside the plausible range cannot be caught by a bound.
    It must time out instead of holding the stream forever."""
    import struct

    parser = PingParser(stall_timeout_s=1.0)
    bogus = bytearray(point_set(1))
    struct.pack_into("<H", bogus, 2, 20_000)     # plausible, but wrong

    assert parser.feed(bytes(bogus), now=100.0) == []
    assert parser.feed(b"", now=100.5) == []     # still within the timeout
    frames = parser.feed(point_set(2), now=102.0)

    assert len(frames) == 1
    assert parser.decode(frames[0]).ping_number == 2


def test_a_slow_link_delivering_a_real_frame_is_not_mistaken_for_a_stall():
    """The stall guard must not punish a genuinely slow link."""
    parser = PingParser(stall_timeout_s=1.0)
    stream = point_set(1)
    assert parser.feed(stream[:20], now=100.0) == []
    frames = parser.feed(stream[20:], now=100.4)
    assert len(frames) == 1
