import unittest

from bioloid import protocol
from bioloid.protocol import ProtocolError


class PacketEncoding(unittest.TestCase):
    def test_ping_matches_datasheet_example(self):
        # ROBOTIS e-Manual: ping to ID 1 is FF FF 01 02 01 FB
        self.assertEqual(protocol.ping(1), bytes.fromhex("FFFF010201FB"))

    def test_write_goal_position(self):
        # Write 0x0200 (512) to goal position (30) of ID 1
        pkt = protocol.write(1, 30, protocol.to_word(512))
        self.assertEqual(pkt, bytes.fromhex("FFFF0105031E0002D6"))

    def test_sync_write_layout(self):
        pkt = protocol.sync_write(30, 2, {1: [0x10, 0x02], 2: [0x20, 0x01]})
        self.assertEqual(pkt[2], protocol.BROADCAST_ID)
        self.assertEqual(pkt[4], protocol.SYNC_WRITE)
        self.assertEqual(list(pkt[5:13]), [30, 2, 1, 0x10, 0x02, 2, 0x20, 0x01])
        self.assertEqual(pkt[-1], protocol.checksum(pkt[2:-1]))

    def test_sync_write_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            protocol.sync_write(30, 2, {1: [1]})

    def test_word_round_trip(self):
        for v in (0, 1, 255, 256, 512, 1023, 65535):
            self.assertEqual(protocol.from_word(*protocol.to_word(v)), v)


class StatusDecoding(unittest.TestCase):
    def test_parse_valid_status(self):
        raw = protocol.build_status(3, 0, [0x20, 0x00])
        st = protocol.parse_status(raw)
        self.assertEqual((st.servo_id, st.error, st.params), (3, 0, [0x20, 0x00]))

    def test_skips_leading_noise(self):
        raw = b"\x00\x13" + protocol.build_status(1)
        self.assertEqual(protocol.parse_status(raw).servo_id, 1)

    def test_bad_checksum(self):
        raw = bytearray(protocol.build_status(1, 0, [5]))
        raw[-1] ^= 0xFF
        with self.assertRaises(ProtocolError):
            protocol.parse_status(bytes(raw))

    def test_error_bits_are_reported(self):
        st = protocol.parse_status(protocol.build_status(7, 0x24))
        self.assertEqual(st.error_names, ["overheating", "overload"])
        with self.assertRaises(ProtocolError):
            st.raise_for_error()


if __name__ == "__main__":
    unittest.main()
