"""tests/robot/test_vision_suite.py: Run the OpenCV tests in tests/robot/vision in a fresh interpreter.

OpenCV and pygame each bundle libSDL2 on macOS; loading both into this test process prints
`objc[...] Class ... is implemented in both` warnings and risks crashes, and pygame is already
loaded here by test_signboard.py. The detector, capture, and inspect tests therefore live in
tests/robot/vision (no __init__.py, so this discovery does not descend into it) and run as one
subprocess; their summary is shown when they fail. They can also be run directly:
`PYTHONPATH=src python -m unittest discover -s tests/robot/vision -v`.
"""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import unittest

VISION_TESTS = Path(__file__).resolve().parent / "vision"
SRC = str(Path(__file__).resolve().parents[2] / "src")


class VisionSuiteTests(unittest.TestCase):
    def test_opencv_suite_passes_in_its_own_interpreter(self):
        if importlib.util.find_spec("cv2") is None:
            self.skipTest("OpenCV is not installed")
        env = {**os.environ, "PYTHONPATH": SRC}
        result = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(VISION_TESTS)],
                                env=env, capture_output=True, timeout=300)
        output = result.stderr.decode(errors="replace")
        self.assertEqual(result.returncode, 0, output)
        self.assertNotIn("objc[", output)
        self.assertIn("\nOK", output)


if __name__ == "__main__":
    unittest.main()
