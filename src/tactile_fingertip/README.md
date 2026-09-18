<!-- src/tactile_fingertip/README.md: Generate and print the single-piece embedded-magnet fingertip. -->
# One-piece magnetic tactile fingertip, v3

**Print one TPU part.** The diamond lattice, contact face, magnet pocket, and sealing roof are fused geometry. There is no separate magnet holder, contact cover, or internal assembly adhesive. Insert the magnet during a print pause, then print the roof over it.

The default envelope is **20 × 20 × 10 mm**. The pocket is exactly **diameter 3 × height 1.5 mm in CAD**, matching the specified magnet. The radial TPU wall, floor, and roof are each **0.8 mm**. Actual fit still depends on printer and magnet tolerances; no loose-clearance allowance is applied by default.

See [research and design](../../docs/magnetic-tactile-fingertip.md) and [current exports](../../3d-models/tactile-fingertip/README.md). The export directory contains only current files, overwritten in place. V3 adds a full backing skin, crossed rib banks, and internal ties.

## Generate and test

From the repository root, using Python 3.12:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r src/tactile_fingertip/requirements.txt
PYTHONPATH=src .venv/bin/python -m tactile_fingertip.generate
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests/tactile_fingertip -v
```

The default destination is `3d-models/tactile-fingertip/`. It contains one printable `fingertip.stl`, `validation.json`. Generation replaces those named files. Use `--output` only for temporary experiments outside the current export directory. Edit the Python source to change dimensions.

Edit dimensions in `config.py`, or supply a JSON override file:

```sh
PYTHONPATH=src .venv/bin/python -m tactile_fingertip.generate --config variant.json --output /tmp/fingertip-variant
```

For example, `{"pocket_wall": 1.0}` changes the surrounding radial wall. Supported wall values are 0.5–1.0 mm. `diameter_compensation` and `depth_compensation` default to zero; small positive values are available only if measured fit requires them. They add to the total diameter/depth, not to each side. Do not scale the STL to adjust magnet fit.

The optional preview uses VTK, included with the tested CadQuery installation, and needs a working graphics context:

```sh
PYTHONPATH=src .venv/bin/python -m tactile_fingertip.preview 3d-models/tactile-fingertip
```

## Geometry and orientation

All coordinates below are **print coordinates**. The flat contact face is on the bed at Z = 0. The lattice joins a continuous mounting skin at Z = 9.2–10.0. Both skins are integral TPU. The upper skin and mid-height ties introduce bridging that must be inspected in the slicer and tested on the printer.

| Feature | Default |
| --- | --- |
| Envelope | 20 × 20 × 10 mm |
| Contact skin / pocket floor | Z = 0–0.8 mm |
| Magnet cavity | Diameter 3 mm; Z = 0.8–2.3 mm |
| Radial pocket wall | 0.8 mm; enclosure outer diameter 4.6 mm |
| Integrated sealing roof | Z = 2.3–3.1 mm; thickness 0.8 mm |
| Magnet center | Print Z = 1.55 mm |
| Diamond outer diagonals | 8.8 × 8.8 mm, nominal 45-degree struts |
| Diamond centers | X = -8, 0, +8; print Z = 4.8 mm |
| In-plane strut thickness | 0.8 mm, normal to inclined edges |
| Rib banks | XZ and YZ; planes at -8, -4, 0, +4, +8; depth 1.2 mm |
| Mounting skin | Full 20 × 20 mm face; Z = 9.2–10.0 |
| Internal ties | Four 17.2 × 1.2 × 0.8 mm beams; X or Y = ±4; Z = 4.4–5.2 |

The two outer columns use inward-facing half-diamonds. This avoids floating upper branches caused by arbitrarily cropping a full diamond. The pocket wall is locally thicker where ribs join it. Crossed ribs and mid-height ties connect the core in both horizontal directions. Geometry has 90-degree rotational symmetry; this does not prove mechanical isotropy, equal stiffness, or fatigue life. The enclosure attaches to the contact skin and moves with it; it does not form a solid pillar to the mounting skin.

For the mounted mechanical frame, rotate the part about X by 180 degrees: X stays the shear axis, mechanical Y reverses, and mechanical Z = 10 minus print Z. The contact face is then mechanical Z = 10, and the outer mounting face is Z = 0. STL remains in its intended print orientation.

## Pause, insert magnet, resume

1. Import `fingertip.stl` in millimeters, without scaling or rotating. Use the flat contact face on the bed.
2. Start with a **0.4 mm nozzle and uniform 0.1 mm layers, including the first layer**. Disable adaptive layers for the documented pause recipe.
3. Inspect the slicer preview. Preserve the internal cavity; do not fill it with infill or internal supports, or remove its inner surface during mesh repair. Fill TPU material regions solid and verify narrow struts survive slicing.
4. Insert a pause **before the first toolpath closing the circular pocket**. With the specified layer schedule this is after completing print Z = **2.3 mm** (23 layers), before the roof layer whose top is Z = **2.4 mm**. UI layer numbering and pause semantics differ; verify the actual toolpath, not only the layer number.
5. At the pause, the head must be parked away using the printer's supported procedure. Insert the axially magnetized diameter 3 × 1.5 mm magnet fully onto the pocket floor, with a recorded pole orientation. Its nominal top is flush with completed Z = 2.3 mm and below the upcoming nozzle path. Check that it stays seated and does not protrude or lift toward the toolhead.
6. Resume printing. The next eight 0.1 mm layers form the 0.8 mm roof, enclosing the magnet. The remaining lattice prints above it. No separate lid or holder is installed.

**STL cannot store a pause.** Set it in the slicer for the actual printer. No printer-specific G-code is included, and no pause command has been tested on the owner's machine. A different first-layer height, layer thickness, scaling, orientation, or adaptive slicing requires recalculating the pause. Default generation rejects a 0.2 mm layer recipe because the 1.5 mm cavity cannot align with its boundaries at the current floor position.

Use the filament's temperature, drying, bed-release and cooling guidance. About 15–25 mm/s is only a trial starting point. Small diamond closures still require physical validation in TPU. During the first trial, verify magnet seating, thermal suitability of the purchased magnet, and successful roof closure; a same-size CAD pocket alone does not establish printed fit.

Prusa documents layer-based pauses and embedding magnets: [pause instructions](https://help.prusa3d.com/article/insert-pause-or-custom-g-code-at-layer_120490) and [embedding examples](https://blog.prusa3d.com/practical-uses-of-color-change-in-prusaslicer_59563/). These establish the general workflow, not firmware compatibility with the owner's printer.

## Mounting and sensing

The mounting skin faces a rigid nonmagnetic backing, while the opposite skin touches the object. The MLX90393 chip faces the cushion from behind the backing and stays fixed to the same fixture. Do not support the board on moving lattice ribs.

An optional **draft rigid base** is exported separately:

```sh
PYTHONPATH=src .venv/bin/python -m tactile_fingertip.mount --output 3d-models/tactile-fingertip
```

`mount-draft.stl` is 34 × 34 mm with a 3 mm backing and two 3 mm standoffs. Print it in a rigid material, not TPU. Its flat underside supports the cushion; the opposite side supports the board on the standoffs. Align the cushion center with the sensor IC. Bond the mounting skin to the backing after checking fit. Adhesive thickness adds directly to sensor spacing. The robot-jaw attachment is not designed yet.

The owner's annotated CJMCU-90393 image states **26.3 × 26 mm**. The selected [Amazon listing](https://www.amazon.co.jp/dp/B0H8H693VS) instead states 20.4 × 20.4 mm; its text is inconsistent with the supplied image. The image is the provisional outline reference, not a verified mechanical drawing. Hole centers are **estimated** at X = ±10, Y = +10 mm relative to the board center. Board thickness 1.6 mm and active-plane offset 0.5 mm are assumptions. Measure the actual board before final manufacture.

The base has 3.2 mm through holes and 6.2 mm, 90-degree countersinks on the cushion side for trial M3 flat-head screws, with washers/nuts behind the PCB. These are chosen fastener clearances, **not a claim about the PCB hole diameter**. Confirm head flushness before bonding, hole spacing/diameter, component and header clearance, board rotation, and nonmagnetic fasteners. Fit the screws before attaching the cushion. Base dimensions can be overridden with `--config` JSON using `MountConfig` fields in `config.py`.

For the cushion alone, `sensor_center_distance = 10` mm remains an exploratory fixture setting, not a fitted assembly value. The draft base places the PCB facing plane 6 mm beyond the cushion backing; the assumed sensing plane is 0.5 mm toward the magnet, giving **13.95 mm magnet-center-to-active-plane distance**, before adhesive thickness. Do not force the 10 mm setting into this base. Verify usable field range and noise at the actual distance before reducing plate/standoff thickness. Neither distance has been tested for sensing performance.

## Checks and limitations

Generation validates one connected CAD solid, exact bounds, a closed cylindrical cavity, radial wall/floor/roof presence, an open accessible pocket at the pause, and an interference-free vertical insertion path at that stage.

An enclosed void creates **two surface shells in STL**: a positive-volume exterior and a negative-volume pocket boundary. These represent one TPU solid with a void, not two printable pieces. Both surfaces must be watertight and correctly oriented; cavity and material volumes are checked against CAD. Do not discard the inner shell as an unwanted loose part.

Ten fingertip regression tests cover exact pocket boundaries, 0.8 mm wall thickness, pause access, both full skins, internal core connectivity and rotational symmetry, bed connectivity at all 100 layer boundaries, invalid settings, 0.5/1.0 mm wall variants, and STL export checks. Printed fit, zero magnet motion, support-free printing, durability, pause behavior, sensor output, and robot installation are not yet physically validated.

Three additional draft-mount tests cover plate/boss integrity, screw access, invalid parameters, and STL export. They do not verify fit to the purchased board.
