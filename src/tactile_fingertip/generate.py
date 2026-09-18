"""Export one-piece TPU fingertip STL with a pause-and-insert manifest."""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

from .config import DEFAULT, OUTPUT_DIRECTORY
from .geometry import make_fingertip
from .validation import validate_export, validate_geometry


def generate(c, output):
    c.validate()
    body = make_fingertip(c)
    checks = validate_geometry(c, body)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # A failed export must not leave an old success report beside new geometry.
    report_path = output / "validation.json"
    report_path.unlink(missing_ok=True)
    stl = output / "fingertip.stl"
    if not body.val().exportStl(str(stl), tolerance=c.stl_tolerance,
                              angularTolerance=c.angular_tolerance, relative=False):
        raise RuntimeError("STL export failed")
    checks.update(validate_export(c, body, stl))
    report = {"design": "v3 crossed diamond ribs with two integral skins", "units": "mm",
              "coordinate_system": "contact face down at print Z=0; mounting skin at Z=height",
              "parameters": asdict(c), "geometry": checks,
              "pause": {"completed_print_z_mm": c.pause_z,
                        "completed_layers_with_uniform_first_layer": round(c.pause_z / c.layer_height),
                        "first_roof_layer_top_z_mm": c.pause_z + c.layer_height,
                        "instruction": "Pause BEFORE the first toolpath closing the pocket; confirm in slicer."},
              "magnet_center_print_z_mm": c.magnet_center_z,
              "sensor_active_plane_print_z_mm": c.sensor_z,
              "physical_validation": "Not printed or sensor-tested; exact CAD fit is not a measured interference fit."}
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIRECTORY)
    parser.add_argument("--config", type=Path, help="JSON overrides for config.py")
    args = parser.parse_args()
    c = replace(DEFAULT, **json.loads(args.config.read_text())) if args.config else DEFAULT
    report = generate(c, args.output)
    print(f"One TPU solid exported: {args.output.resolve()}")
    print(f"STL watertight; pause after Z={report['pause']['completed_print_z_mm']:.2f} mm")


if __name__ == "__main__":
    main()
