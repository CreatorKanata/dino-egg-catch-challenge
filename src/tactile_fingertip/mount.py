"""Draft rigid backing and PCB standoffs; estimated holes require measurement."""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

import cadquery as cq
import trimesh

from .config import DEFAULT, MOUNT_DEFAULT, OUTPUT_DIRECTORY
from .geometry import cylinder, slab


def make_mount(c):
    c.validate()
    body = slab(c.width, c.depth, c.plate)
    for x, y in c.hole_centers:
        body = body.union(cylinder(c.boss_diameter, c.standoff, c.plate).translate((x, y, 0)))
        shaft = cylinder(c.screw_clearance, c.plate+c.standoff).translate((x, y, 0))
        # 90-degree countersink faces the cushion so a flush head avoids interference.
        head = (cq.Workplane("XY").circle(c.head_diameter/2)
                .workplane(offset=(c.head_diameter-c.screw_clearance)/2)
                .circle(c.screw_clearance/2).loft().translate((x, y, 0)))
        body = body.cut(shaft).cut(head)
    return body.clean()


def generate_mount(c, output):
    body = make_mount(c)
    if not body.val().isValid() or len(body.val().Solids()) != 1:
        raise ValueError("Mount must be one valid solid")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "mount-draft.json"
    report_path.unlink(missing_ok=True)
    stl = output / "mount-draft.stl"
    if not body.val().exportStl(str(stl), tolerance=DEFAULT.stl_tolerance, relative=False):
        raise ValueError("Mount STL export failed")
    mesh = trimesh.load_mesh(stl)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError("Invalid mount mesh")
    report = {"status": "Draft fit coupon; board holes and component clearances unmeasured",
              "units": "mm", "parameters": asdict(c), "cad_valid": True,
              "stl_watertight": True,
              "assembly": "Translate mount by fingertip height in print coordinates",
              "nominal_magnet_to_sensor_mm": DEFAULT.height+c.plate+c.standoff
                  -c.chip_active_offset-DEFAULT.magnet_center_z,
              "outline_evidence": "Owner-supplied annotated image: 26.3 x 26 mm",
              "estimated_parameters": ["hole_pitch", "hole_y"],
              "assumed_parameters": ["board_thickness", "chip_active_offset"],
              "fastener": "Trial M3 90-degree flat-head screws, rear washers/nuts; verify actual dimensions"}
    report_path.write_text(json.dumps(report, indent=2)+"\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIRECTORY)
    parser.add_argument("--config", type=Path, help="MountConfig JSON overrides")
    args = parser.parse_args()
    c = replace(MOUNT_DEFAULT, **json.loads(args.config.read_text())) if args.config else MOUNT_DEFAULT
    generate_mount(c, args.output)
