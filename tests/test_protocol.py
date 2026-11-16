"""Protocol checks against byte sequences observed on the USB bus with real hardware."""
from wscanfd import protocol as p

h = bytes.fromhex


def test_pack_matches_capture():
    assert p.pack(p.Cmd.DEVICE_INFO) == h("beef000001018005c0b7dead")
    assert p.pack(p.Cmd.START_CAN, p.start_payload(0)) == h("beef00040101800355800300" "1937dead")
    assert p.pack(p.Cmd.RESET_CAN, p.reset_payload(0)) == h("beef00080101800955092900000000003ed0dead")
    assert p.pack(p.Cmd.CLOSE, p.close_payload()) == h("beef00020101800a550a15cddead")
    assert p.pack(p.Cmd.FILTER, p.clear_filter_payload(0)) == h("beef00040101800b7e000fff4ec6dead")


def test_init_prefix_matches_capture():
    pkt = p.pack(p.Cmd.INIT_CAN, p.init_payload(0, p.timing_word(250_000)))
    assert pkt[:32] == h("beef002001018002550200010001000000" "00ffffffff000000000014be2f0303")


def test_transmit_prefix_matches_capture():
    f = p.Frame(0x123, bytes(range(8)))
    pkt = p.pack(p.Cmd.TRANSMIT, p.transmit_payload(f))
    assert len(pkt) == 38
    assert pkt[:32] == h("beef001a0101800455f100000000000001230800010203040506070000000000")


def test_unpack_replies():
    r = p.unpack(h("beef00020101800b7f003fc5dead"))
    assert r.cmd == p.Cmd.FILTER and r.payload == h("7f00")
    assert p.unpack(h("beef000001018002c0b1dead")).cmd == p.Cmd.INIT_CAN
    assert p.unpack(h("beef000201018009" "0000c0c1dead")).payload == b"\0\0"


def test_parse_rx_classic():
    buf = h("c00f801f000004ff0000000d0000000055d4d76300"
            "48029004000008fffffefdfcfbfaf9f85543d86300")
    a, b = p.parse_rx(buf, channel=0)
    assert (a.can_id, a.data, a.extended, a.fd) == (0x7E0, h("0000000d"), False, False)
    assert a.timestamp_us == 0x0063D7D4 * 100
    assert (b.can_id, b.data) == (0x124, h("fffefdfcfbfaf9f8"))


def test_parse_rx_idle():
    assert p.parse_rx(bytes(8)) == []


def test_timing_reference_table_consistent():
    for rate, word in {**p.REF_ABIT}.items():
        assert round(p.BitTiming.from_word(word).bitrate()) == rate
    for rate, word in p.REF_DBIT.items():
        assert round(p.BitTiming.from_word(word).bitrate()) == rate


def test_calc_timing():
    for rate in (33_333, 83_333, 125_000, 250_000, 500_000, 1_000_000):
        t = p.calc_timing(rate)
        assert abs(t.bitrate() - rate) / rate < 1e-3
        assert 0.75 <= t.sample_point() <= 0.85
    t = p.calc_timing(2_000_000, data=True)
    assert t.bitrate() == 2_000_000



def test_status_payload():
    # cmd 0x800D, payload 55 0D 29 <dev> <ch> 00 00 00
    assert p.status_payload(1) == h("550d290001000000")
    assert p.pack(p.Cmd.GET_STATUS, p.status_payload(0))[6:8] == h("800d")


def test_fd_transmit_layout():
    f = p.Frame(0x1ABCDEF0, bytes(range(20)), extended=True, fd=True, brs=True, channel=1)
    b = p.transmit_payload(f)
    assert len(b) == 0x56 and b[:2] == h("55f2")
    assert b[4] == 1 and b[6:10] == h("1abcdef0") and b[10] == 20 and b[11] == 1 and b[12] == 1 and b[13] == 1
    assert b[14:34] == bytes(range(20))
