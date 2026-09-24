"""tests/robot/test_detect_timing.py: Checks of the one-time egg-detector timing log.

The loop measures the detector over its first frames and logs the average once at INFO; the
frozen DetectTiming state and the single log line are verified here without OpenCV.
"""

import unittest

from robot.vision.timing import DetectTiming, record_detect_time


class DetectTimingTests(unittest.TestCase):
    def test_logs_average_once_at_window(self):
        timing = DetectTiming()
        for _ in range(2):
            timing = record_detect_time(timing, 0.004, window=3)
        with self.assertLogs("robot.vision.timing", level="INFO") as logs:
            timing = record_detect_time(timing, 0.004, window=3)
        self.assertEqual(len(logs.output), 1)
        self.assertIn("4.0 ms per frame (average of 3 frames)", logs.output[0])
        self.assertIs(record_detect_time(timing, 1.0, window=3), timing)  # no further updates

    def test_does_not_mutate_and_ignores_negative_time(self):
        start = DetectTiming()
        updated = record_detect_time(start, -1.0, window=5)
        self.assertEqual((start, updated), (DetectTiming(), DetectTiming(frames=1, total_s=0.0)))


if __name__ == "__main__":
    unittest.main()
