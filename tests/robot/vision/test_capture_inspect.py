"""tests/robot/vision/test_capture_inspect.py: Checks of the capture writer and the inspect CLI.

save_capture() must write one PNG per available camera plus a JSON record (detections, mode)
into a temporary directory without overwriting same-second captures; the inspect tool must
print the detections of a synthetic egg image, handle `--rgb`, and write `<image>-detected.png`.
Runs in its own interpreter with OpenCV (see tests/robot/test_vision_suite.py).
"""

import contextlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import cv2
import numpy as np

from robot.vision import inspect as inspect_tool
from robot.vision.capture import save_capture
from robot.vision.egg_size import EggDetection

EGG = EggDetection(cx=0.5, cy=0.55, w=0.4, h=0.6, color="green", spots=3, area_px=40000)
STAMP_TIME = time.strptime("2026-09-24 12:34:56", "%Y-%m-%d %H:%M:%S")


def egg_image(spot_bgr=(40, 160, 60)):
    """An egg with the measured proportions (spot diameter ~1/4 of the egg height)."""
    frame = np.full((480, 640, 3), (110, 65, 30), dtype=np.uint8)  # blue tarp color: no spot class
    cv2.ellipse(frame, (320, 250), (130, 100), 0, 0, 360, (245, 245, 245), -1)
    for center in ((250, 255), (390, 240), (320, 190), (320, 305), (325, 250)):
        cv2.circle(frame, center, 25, spot_bgr, -1)
    return frame


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)

    def test_writes_pngs_and_json(self):
        frame = egg_image()
        path = save_capture({"front": frame, "wrist": None, "top": frame}, (EGG,), "manual",
                            self.folder / "captures", STAMP_TIME)
        self.assertEqual(path.name, "20260924-123456.json")
        record = json.loads(path.read_text())
        self.assertEqual((record["mode"], record["frames"]), ("manual", ["front", "top"]))
        self.assertEqual(record["detections"][0]["color"], "green")
        self.assertEqual(record["detections"][0]["size"], "ok")
        front = cv2.imread(str(path.with_name("20260924-123456-front.png")))
        np.testing.assert_array_equal(front, frame)  # lossless, still BGR
        self.assertFalse(path.with_name("20260924-123456-wrist.png").exists())

    def test_same_second_capture_gets_a_suffix(self):
        first = save_capture({"front": egg_image()}, (), "fsc", self.folder, STAMP_TIME)
        second = save_capture({"front": egg_image()}, (), "fsc", self.folder, STAMP_TIME)
        self.assertEqual((first.name, second.name), ("20260924-123456.json", "20260924-123456-1.json"))
        self.assertTrue((self.folder / "20260924-123456-1-front.png").exists())


class InspectToolTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)

    def run_tool(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = inspect_tool.main([str(arg) for arg in args])
        return code, out.getvalue()

    def test_prints_detections_and_writes_annotated_image(self):
        image = self.folder / "front.png"
        cv2.imwrite(str(image), egg_image())
        code, output = self.run_tool(image)
        self.assertEqual(code, 0)
        self.assertIn("1 egg(s)", output)
        self.assertIn("1: green egg", output)
        self.assertIn("size=ok", output)
        self.assertTrue((self.folder / "front-detected.png").exists())

    def test_debug_lists_candidates_with_verdicts(self):
        image = self.folder / "front.png"
        cv2.imwrite(str(image), egg_image())
        code, output = self.run_tool(image, "--debug")
        self.assertEqual(code, 0)
        self.assertIn("spot cluster(s):", output)
        self.assertIn("d_med=", output)
        self.assertIn("EGG", output)
        self.assertIn("green=", output)
        self.assertIn("ellipse=(", output)

    def test_rgb_flag_swaps_channels_first(self):
        image = self.folder / "screenshot.png"
        cv2.imwrite(str(image), egg_image((40, 40, 220))[..., ::-1])  # red spots, RGB-ordered like an old screenshot
        self.assertNotIn("red egg", self.run_tool(image)[1])  # read as BGR the spots look blue (no class)
        self.assertIn("1: red egg", self.run_tool(image, "--rgb")[1])

    def test_unreadable_image(self):
        self.assertEqual(self.run_tool(self.folder / "missing.png")[0], 2)


if __name__ == "__main__":
    unittest.main()
