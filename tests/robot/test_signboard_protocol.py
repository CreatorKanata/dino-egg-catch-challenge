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
from robot.display_status import DisplayStatus, Overlay
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
OVERLAYS = (Overlay("front", 0.56, 0.57, 0.64, 0.69, "target", "place the egg here"),
            Overlay("front", 0.5, 0.5, 0.2, 0.3, "egg_out", "red egg: too far"),
            Overlay("front", -0.05, 0.5, 1.2, 0.4, "egg_ok", "green egg", 92.5))  # fitted, past the edge


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
                                            "stopped": False, "notice_level": "info", "recording_s": None,
                                            "progress": None, "overlays": []})
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

    def test_overlays_and_notice_level_round_trip(self):
        status = replace(STATUS, notice="Egg too far", notice_level="warning", overlays=OVERLAYS)
        plain = DisplayPacket(drive=DRIVE, controller=CONTROLLER, frames=(), status=status)
        self.assertEqual(read_packet(io.BytesIO(encode(plain))).status, status)

    def test_malformed_overlays_return_none(self):
        status = replace(STATUS, overlays=OVERLAYS[:1])
        good = json.loads(encode(DisplayPacket(DRIVE, CONTROLLER, (), status)).partition(b"\n")[0])
        overlay = good["status"]["overlays"][0]
        bad_overlays = {
            "not a list": "x",
            "too many": [overlay] * 9,
            "not an object": [1],
            "bad kind": [{**overlay, "kind": "star"}],
            "bad camera": [{**overlay, "camera": 3}],
            "long label": [{**overlay, "label": "x" * 65}],
            "out of range": [{**overlay, "cx": 2.5}],
            "bad angle": [{**overlay, "angle": 400.0}],
            "nan angle": [{**overlay, "angle": float("nan")}],
            "bool number": [{**overlay, "w": True}],
            "missing number": [{key: value for key, value in overlay.items() if key != "h"}],
        }
        cases = {name: {**good["status"], "overlays": value} for name, value in bad_overlays.items()}
        cases["bad notice level"] = {**good["status"], "notice_level": "loud"}
        cases["missing overlays"] = {k: v for k, v in good["status"].items() if k != "overlays"}
        for name, fields in cases.items():
            with self.subTest(name=name):
                line = json.dumps({**good, "status": fields}).encode() + b"\n"
                self.assertIsNone(read_packet(io.BytesIO(line)))

    def test_recording_progress_and_rect_overlays_round_trip(self):
        rects = (Overlay("front", 0.52, 0.42, 0.78, 0.68, "basket", "basket"),
                 Overlay("front", 0.53, 0.41, 0.8, 0.7, "release_target", ""))
        status = replace(STATUS, action="auto_release", arm_status="auto release", recording_s=4.2, progress=0.5,
                         overlays=rects)
        self.assertEqual(read_packet(io.BytesIO(encode(DisplayPacket(DRIVE, CONTROLLER, (), status)))).status, status)

    def test_malformed_optional_numbers_return_none(self):
        good = json.loads(encode(packet(())).partition(b"\n")[0])
        cases = {"negative recording": {"recording_s": -1.0}, "progress above one": {"progress": 1.5},
                 "text progress": {"progress": "half"}, "bool recording": {"recording_s": True},
                 "nan progress": {"progress": float("nan")}}
        for name, fields in cases.items():
            with self.subTest(name=name):
                line = json.dumps({**good, "status": {**good["status"], **fields}}).encode() + b"\n"
                self.assertIsNone(read_packet(io.BytesIO(line)))
        missing = {key: value for key, value in good["status"].items() if key != "progress"}
        self.assertIsNone(read_packet(io.BytesIO(json.dumps({**good, "status": missing}).encode() + b"\n")))

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
