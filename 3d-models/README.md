<!-- 3d-models/README.md: Separate reference geometry from future custom dinosaur designs. -->
# 3D Models

`so101/` contains the existing SO101 assembly and component STEP files used as reference geometry. Their presence does not establish that custom dinosaur parts have been designed or physically validated.

Preserve these reference files. Add custom dinosaur covers, head/jaw parts, mounts, controller enclosures, and egg-related parts in clearly named directories as they are developed. Keep editable sources and fabrication exports distinguishable, and document units, material, mounting, compatible hardware, and physical validation for each release.

Use actual STEP geometry and measured hardware for fitting. Generated appearance images are design references, not dimensional evidence. See the [concept](../docs/concept.md) for appearance constraints and fabrication proposals.

## Magnetic tactile fingertip

See [tactile-fingertip/](tactile-fingertip/README.md) for the original 20 × 20 × 10 mm FDM cushion, designed around one diameter 3 × height 1.5 mm magnet. STL exports and digital validation are available; printing, bonding, sensing, and robot mounting remain unverified. The editable source is under [src/tactile_fingertip/](../src/tactile_fingertip/README.md).
