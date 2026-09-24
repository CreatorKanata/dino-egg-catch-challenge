"""tests/robot/test_align_trace.py: Checks of the per-frame alignment CSV trace writer.

append_row() must create the file with a header on the first row, append one line per call, and
leave missing detections as empty cells; trace_path() names the file after the start time. Uses a
temporary directory, never the repository's captures/.
"""

import csv
from pathlib import Path
import shutil
import tempfile
import time
import unittest

from robot.align import TraceRow
from robot.vision.align_trace import TRACE_COLUMNS, append_row, trace_path

ROW = TraceRow(t=0.5, cx_raw=0.61, h_raw=0.4, top_raw=0.25, cx_smooth=0.6, top_smooth=0.24, x_vel=0.01,
               y_vel=-0.02, result="running")


class AlignTraceTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)

    def test_header_once_then_one_line_per_row(self):
        path = self.folder / "sub" / "x-align.csv"
        append_row(path, ROW)
        append_row(path, TraceRow(1.0, None, None, None, 0.6, 0.24, 0.0, 0.0, "lost"))
        with path.open(newline="") as handle:
            rows = list(csv.reader(handle))
        self.assertEqual(rows[0], list(TRACE_COLUMNS))
        self.assertEqual(rows[0][2:6], ["h_raw", "top_raw", "cx_smooth", "top_smooth"])
        self.assertEqual(rows[1], ["0.5000", "0.6100", "0.4000", "0.2500", "0.6000", "0.2400", "0.0100", "-0.0200",
                                   "running"])
        self.assertEqual(rows[2][:4] + rows[2][-1:], ["1.0000", "", "", "", "lost"])
        self.assertEqual(len(rows), 3)

    def test_path_is_stamped(self):
        stamp = time.strptime("2026-09-24 22:10:05", "%Y-%m-%d %H:%M:%S")
        self.assertEqual(trace_path(self.folder, stamp), self.folder / "20260924-221005-align.csv")


if __name__ == "__main__":
    unittest.main()
