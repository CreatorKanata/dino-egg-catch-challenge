"""tests/robot/test_signboard_protocol.py: Round-trip checks of the parent -> signboard pipe format.

The signboard child process rebuilds Manual Mode state, mode status, and BGR frames from these
packets, and reports KachiButton commands back as JSON lines, so encoding, decoding, and
rejection of truncated or malformed data are verified in memory.
"""

from dataclasses import replace
import io
import json
import unittest

import numpy as np

from robot.dino_controller_reader import INITIAL_STATE
from robot.display_status import DisplayStatus
from robot.drive_state import DriveState
from robot.signboard_protocol import (
    CLOSE_MESSAGE,
    CloseRequest,
    DisplayPacket,
    encode,
    encode_command,
    parse_command_line,
    read_packet,
)

DRIVE = DriveState(speed_index=2, catch_requested=True, input_lost=False, pending_rotation_deg=-12.5)
CONTROLLER = replace(INITIAL_STATE, synchronized=True, up=True, right=True, button=True,
                     last_seq=7, last_update_monotonic=3.5)
FRAME = np.arange(4 * 3 * 3, dtype=np.uint8).reshape(3, 4, 3)
STATUS = DisplayStatus(mode="fsc", action="none", voice_listening=True, notice="Listening...",
                       arm_status="leader fault")


def packet(frames=(("top", FRAME), ("front", None))):
    return DisplayPacket(drive=DRIVE, controller=CONTROLLER, frames=frames, status=STATUS)


class ProtocolTests(unittest.TestCase):
    def test_round_trip(self):
        decoded = read_packet(io.BytesIO(encode(packet())))
        self.assertEqual(decoded.drive, DRIVE)
        self.assertEqual(decoded.status, STATUS)
        # Sequence bookkeeping is not transmitted; only the displayed levels are.
        self.assertEqual(decoded.controller, replace(CONTROLLER, last_seq=None, last_update_monotonic=None))
        (top_name, top), (front_name, front) = decoded.frames
        self.assertEqual((top_name, front_name), ("top", "front"))
        np.testing.assert_array_equal(top, FRAME)
        self.assertIsNone(front)

    def test_header_format(self):
        header, _, body = encode(packet()).partition(b"\n")
        fields = json.loads(header)
        self.assertEqual(fields["v"], 1)
        self.assertEqual(fields["status"], {"mode": "fsc", "action": "none", "voice_listening": True,
                                            "notice": "Listening...", "arm_status": "leader fault",
                                            "stopped": False})
        self.assertEqual(fields["frames"], [{"name": "top", "w": 4, "h": 3}, {"name": "front", "w": 0, "h": 0}])
        self.assertEqual(body, FRAME.tobytes())

    def test_non_contiguous_frame_is_sent_in_display_order(self):
        flipped = FRAME[:, ::-1]
        decoded = read_packet(io.BytesIO(encode(packet((("top", flipped),)))))
        np.testing.assert_array_equal(decoded.frames[0][1], flipped)

    def test_consecutive_packets_share_a_stream(self):
        stream = io.BytesIO(encode(packet()) + encode(packet((("wrist", FRAME),))) + CLOSE_MESSAGE)
        self.assertEqual(read_packet(stream).frames[0][0], "top")
        self.assertEqual(read_packet(stream).frames[0][0], "wrist")
        self.assertIsInstance(read_packet(stream), CloseRequest)
        self.assertIsNone(read_packet(stream))

    def test_wrong_frame_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            encode(packet((("top", np.zeros((3, 4), dtype=np.uint8)),)))

    def test_close_message(self):
        self.assertEqual(encode(CloseRequest()), CLOSE_MESSAGE)
        self.assertIsInstance(read_packet(io.BytesIO(CLOSE_MESSAGE)), CloseRequest)

    def test_malformed_headers_return_none(self):
        good = json.loads(encode(packet(())).partition(b"\n")[0])
        cases = {
            "empty stream": b"",
            "not json": b"hello\n",
            "not an object": b"[1]\n",
            "wrong version": json.dumps({**good, "v": 2}).encode() + b"\n",
            "missing drive": json.dumps({k: v for k, v in good.items() if k != "drive"}).encode() + b"\n",
            "bad drive type": json.dumps({**good, "drive": {**good["drive"], "input_lost": 1}}).encode() + b"\n",
            "bad rotation": json.dumps({**good, "drive": {**good["drive"], "pending_rotation_deg": "x"}}).encode() + b"\n",
            "nan rotation": json.dumps({**good, "drive": {**good["drive"], "pending_rotation_deg": float("nan")}}).encode() + b"\n",
            "bad frame entry": json.dumps({**good, "frames": [{"name": "top", "w": -1, "h": 3}]}).encode() + b"\n",
            "half-empty frame": json.dumps({**good, "frames": [{"name": "top", "w": 4, "h": 0}]}).encode() + b"\n",
            "missing status": json.dumps({k: v for k, v in good.items() if k != "status"}).encode() + b"\n",
            "bad mode": json.dumps({**good, "status": {**good["status"], "mode": "auto"}}).encode() + b"\n",
            "bad action": json.dumps({**good, "status": {**good["status"], "action": 1}}).encode() + b"\n",
            "bad voice": json.dumps({**good, "status": {**good["status"], "voice_listening": 0}}).encode() + b"\n",
            "bad notice": json.dumps({**good, "status": {**good["status"], "notice": None}}).encode() + b"\n",
            "long notice": json.dumps({**good, "status": {**good["status"], "notice": "x" * 999}}).encode() + b"\n",
            "bad stopped": json.dumps({**good, "status": {**good["status"], "stopped": "no"}}).encode() + b"\n",
            "bad arm status": json.dumps({**good, "status": {**good["status"], "arm_status": "on"}}).encode() + b"\n",
            "unknown type": b'{"v": 1, "type": "reset"}\n',
            "command on stdin": encode_command("stop"),
            "no newline": json.dumps(good).encode(),
        }
        for name, data in cases.items():
            with self.subTest(name=name):
                self.assertIsNone(read_packet(io.BytesIO(data)))

    def test_default_status_round_trip(self):
        plain = DisplayPacket(drive=DRIVE, controller=CONTROLLER, frames=())
        self.assertEqual(read_packet(io.BytesIO(encode(plain))).status, DisplayStatus())

    def test_command_line_round_trip(self):
        line = encode_command("mode_toggle")
        self.assertEqual(json.loads(line), {"v": 1, "type": "command", "command": "mode_toggle"})
        self.assertTrue(line.endswith(b"\n"))
        self.assertEqual(parse_command_line(line), "mode_toggle")

    def test_malformed_command_lines_return_none(self):
        cases = (b"", b"hello\n", b'{"v": 1, "type": "command"}\n', b'{"v": 1, "type": "command", "command": 3}\n',
                 b'{"v": 1, "type": "command", "command": ""}\n', b'{"v": 2, "type": "command", "command": "hi"}\n',
                 CLOSE_MESSAGE, encode_command("x" * 65), encode_command("hi")[:-1])
        for line in cases:
            with self.subTest(line=line):
                self.assertIsNone(parse_command_line(line))

    def test_short_read_returns_none(self):
        data = encode(packet())
        self.assertIsNone(read_packet(io.BytesIO(data[:-1])))


if __name__ == "__main__":
    unittest.main()
