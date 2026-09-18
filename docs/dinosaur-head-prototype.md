<!-- docs/dinosaur-head-prototype.md: Current rounded-head design, mounting references, and CAD validation limits. -->
# Rounded Dinosaur Head Prototype

Updated in Autodesk Fusion on 2026-09-07 using the owner's current closed pose in `SO101 Assembly`. That pose was preserved. The original SO101 component geometry was not edited.

## Current files and appearance

The Fusion `dino-egg-catch-challenge` folder contains two linked source designs:

- `Dino_Upper_Jaw`: outer sage-green head with a continuous rounded skull-to-snout contour, larger rearward eyes, visible nostril openings, and two rounded dorsal bumps. Fits `Moving_Jaw_SO101`.
- `Dino_Lower_Jaw`: a slender cream lower jaw and rounded recessed cheeks. Fits `Wrist_Roll_Follower_SO101`. The former large box-like motor shroud has been removed.

The green cheek now extends continuously from the skull around the outside of the lower-jaw root in the owner's closed pose. Its lower boundary reaches Z = -50.5 mm in the upper-shell frame. The hidden upper edge of the cream root is recessed to Z = -26.5 mm in that same pose frame, removing the reported cover-to-cover collision while retaining all four lower mounting bores. The source hardware constrains the profile, and the result is an interpretation of the supplied appearance reference, not an exact reproduction. Some mechanism remains visible from the mouth and rear. The decorative spots and teeth in the appearance reference are not included.

Native backups are `3d-models/dino-so101-customize/Dino_Upper_Jaw.f3d` and `Dino_Lower_Jaw.f3d`. Each source contains one connected printable solid plus a hidden SO101 reference body named `REFERENCE ... Do not print`. Export only the body named `... - Print` for fabrication. Eye regions are integrated geometry with paint-guide face colors, not separate inserts.

`SO101 Assembly` contains linked occurrences of both source designs, rigidly grouped with the corresponding original mounting parts. Embedded reference bodies are hidden in the assembly to avoid duplicate visible hardware. The two external references were refreshed after this revision.

The final shapes are editable BRep base features. Loft geometry was used to form the green contour, but the final files do not provide a fully dimension-driven sketch/loft history.

## Mounting

Each shell retains four modeled 1.5 mm bores: two stations, one bore on each side. The locations derive from the existing reference holes.

| Shell | Station | Source-native axis position, mm | Axis direction |
| --- | --- | --- | --- |
| Upper | Rear | X = 1.972983, Y = -28.831369 | Z |
| Upper | Front | X = -0.134445, Y = -48.720028 | Z |
| Lower | Rear | X = -27.463839, Z = 50.742160 | Y |
| Lower | Front | X = -17.203235, Z = 78.932938 | Y |

The upper boss ends follow the outer head surface, reducing the former protruding circular pads. The lower cheeks and jaw rim are joined with curved supports. Fastener type, screw length, engagement, tool access, pilot-hole suitability, and printer compensation remain unverified.

The green inner loft uses a nominal 2.5 mm reduction in section radii. The lower rim uses a nominal 2.3 mm radius difference, and its cheeks use 2.5 mm. These are construction dimensions, not a measured minimum wall thickness. Subsequent clearance cuts affect local thickness.

## Coordinates and wrist opening

Source bodies were rigidly reoriented without scaling in the saved reference bodies:

- Upper: shell X = -source Y; shell Y = -source Z; shell Z = source X.
- Lower: shell X = sin(20 degrees) * source X + cos(20 degrees) * source Z; shell Y = -source Y; shell Z = cos(20 degrees) * source X - sin(20 degrees) * source Z.

The shell occurrences use the inverse mapping followed by the corresponding assembly occurrence transform. The rounded lower cheeks were shaped in the upper shell's closed-pose coordinate frame and transformed back into the lower source frame.

The lower rear remains cut at source-native Z = 7.5 mm. The `Wrist_Roll_Pitch_SO100` housing reaches Z = 5.228 mm in that frame, leaving a 2.272 mm separation between axial bounding planes. The rounded revision has a larger actual surface gap: approximately 15.58 mm in the displayed pose. This does not certify cable clearance or every wrist orientation.

## CAD validation

The final source shapes passed these checks:

- One closed solid and one connected lump per printable body.
- Four cylindrical mounting bores per shell, each 1.5 mm in diameter.
- Zero Boolean overlap volume with each shell's original SO101 reference.
- Zero Boolean overlap volume with the original assembly bodies in the owner's current closed pose. The inserted dinosaur occurrences were excluded from that obstacle list to avoid self-comparisons.
- Zero upper-to-lower overlap volume in that pose; measured minimum cover-to-cover separation is approximately 1.550 mm.

| Property | Upper | Lower |
| --- | ---: | ---: |
| CAD material volume | 42.338 cm3 | 12.623 cm3 |
| Solid PLA estimate at 1.24 g/cm3 | 52.50 g | 15.65 g |
| Minimum own-reference separation | 0.288 mm | Approximately zero: local contact remains |

The lower source has no detected volume penetration but still has local surface contact with its reference, including near the frame. Additional relief was added around the finger transition; this did not eliminate every contact point. Do not interpret zero overlap as verified print clearance. Resolve those contact areas and confirm fit before fabrication qualification.

Mass values are density-based estimates, not measured print masses. Slicer settings, paint, supports, and fasteners change actual mass.

No motion limits or joints were present in the original assembly. This revision is checked at the owner's closed pose only. The earlier box-shroud construction sweep does not validate these redesigned shapes. Continuous jaw travel, all arm/wrist poses, installation sequence, minimum wall thickness, egg gripping, and physical fit remain unverified. No physical hardware was moved.
