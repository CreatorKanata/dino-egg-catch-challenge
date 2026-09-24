"""src/robot/vision/align_trace.py: Per-frame CSV trace of an Auto Catch alignment, for tuning.

While aligning, the loop appends one row per frame (time since start, raw egg cx, ellipse width,
bbox height, and top edge, smoothed cx and ellipse width, commanded x/y velocities, result) to `captures/<YYYYmmdd-HHMMSS>-align.csv`, stamped at the
alignment start. Each call opens the file in append mode and writes one line, so the loop is
never blocked for longer than that. Stdlib-only; the directory is git-ignored (CAPTURE_DIR).
"""

import csv
from pathlib import Path
import time

from robot.align import TraceRow
from robot.config import CAPTURE_DIR

TRACE_COLUMNS = ("t", "cx_raw", "w_raw", "h_raw", "top_raw", "cx_smooth", "w_smooth", "x_vel", "y_vel", "result")
STAMP_FORMAT = "%Y%m%d-%H%M%S"


def trace_path(directory: str | Path = CAPTURE_DIR, local_time: time.struct_time | None = None) -> Path:
    """`<directory>/<stamp>-align.csv` for an alignment starting now (or at `local_time`)."""
    return Path(directory) / f"{time.strftime(STAMP_FORMAT, local_time or time.localtime())}-align.csv"


def _cell(value: float | str | None) -> str:
    if value is None:
        return ""
    return value if isinstance(value, str) else f"{value:.4f}"


def append_row(path: Path, row: TraceRow) -> None:
    """Append one row, writing the header first when the file is new. Raises OSError on failure."""
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists()
    with path.open("a", newline="") as handle:
        writer = csv.writer(handle)
        if new_file:
            writer.writerow(TRACE_COLUMNS)
        writer.writerow([_cell(getattr(row, column)) for column in TRACE_COLUMNS])
