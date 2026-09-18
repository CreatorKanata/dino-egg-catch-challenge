<!-- docs/magnetic-tactile-fingertip.md: Research and proposed FDM prototype; separate evidence from unvalidated design choices. -->
# Magnetic Tactile Fingertip: Research and Prototype Plan

Updated: 2026-09-18. Status: v3 crossed-lattice CAD implemented and exported; physical validation pending.

This document describes the proposed REHC2026 experiment for one robot gripper finger. It records the public sources reviewed, the owner's latest dimensions, a concrete starting geometry, and the checks required before calling the prototype functional. The one-piece v3 has parametric source, STL exports, and automated geometry checks. Printing, pause execution, magnet fit, force characterization, sensor acquisition, and classification remain unvalidated. See the [generator instructions](../src/tactile_fingertip/README.md) and [export validation record](../3d-models/tactile-fingertip/README.md). Competition rules and submission requirements are outside this investigation.

## 1. Current requirements and scope

| Item | Requirement or status |
| --- | --- |
| Application | One tactile finger on the robot gripper; opposite finger may remain passive |
| Minimum sensing hardware | One permanent magnet and one MLX90393 three-axis magnetic sensor |
| Magnet | Owner-specified cylindrical neodymium magnet, diameter 3 mm, height 1.5 mm |
| Soft envelope | Owner's approximate target: 20 × 20 × 10 mm; supersedes the earlier 20–30 mm footprint and 10–20 mm thickness range |
| Material/process | Flexible TPU/TPE, household FDM printer; pause to embed the magnet |
| Latest owner decision | One TPU part with both skins and depth connections; pocket walls 0.5–1.0 mm; no separate holder or cover |
| Internal structure | Explicit, editable flexible lattice; diamond or gyroid considered |
| Electronics | Sensor PCB mechanically separate from the soft part |
| Intended analysis | Normal Press, Left Shear, Right Shear, and exploratory Slip detection using Solist-AI / ML63Q2557 |
| Source location | Generator under `src/tactile_fingertip/` |
| Exports | Printable STL; editable parametric source |
| Current deliverable | Parametric source, one printable STL part, preview, and tests |

The 10 mm target describes the one-piece soft cushion, including its integral contact skin and pocket enclosure, but excludes a rigid mount, PCB, and any sensor spacing adjustment. Final gripper installation depth will therefore be greater than 10 mm. No existing SO101 or dinosaur-jaw reference geometry will be modified for the initial coupon.

## 2. Research findings

### 2.1 Reference paper

Bang et al., *Magnet-Based Soft Robotic Skin Using a 3D-Printed Multi-Lattice Structure and CNN-Based Tactile Super-Resolution*, describes TPU lattices, embedded magnets, distributed Hall sensors, and CNN estimation of contact location and normal force. The reviewed six-page paper uses SLS, not household FDM. Its surface, diamond-strut infill, and base structures distribute local deformation across several sensors. [R1, Section II]

The reported fabrication uses a Formlabs Fuse 1+ 30W and TPU 90A powder. The sensing experiment uses multiple MLX90393 sensors, rather than a single fingertip sensor. [R1, Sections II–IV]

The transferable ideas are mechanical conversion of contact into magnetic-field changes, tunable compliance, and physical separation of electronics from the contact surface. The paper does not validate this proposed FDM geometry, single-sensor slip classification, or its performance on Solist-AI.

No downloadable CAD release was verified in the reviewed sources. This is a limited research finding, not proof that no public design files exist. The new geometry will be authored independently.

### 2.2 Related thesis

DGIST's repository lists Yunseong Bang's 2026 master's thesis, *Magnet-Based Soft Robotic Skin with Multi-Lattice Structure for Tactile Super-Resolution via Sim-to-Real Transfer*. The catalog includes hardware, fabrication, CNN regression, and simulation-to-real topics. The catalog and contents listing were inspected; the full thesis was not reviewed. No geometry or fabrication numbers are taken from it. [R2]

### 2.3 MLX90393 constraints

MLX90393 reports magnetic-field components, not direct XYZ displacement or force. Displacement inference requires calibration and assumptions about magnet orientation. Its axes are acquired sequentially; conversion time depends on filtering and oversampling. Sampling rate must be measured for the chosen settings. [R3, Sections 12–15]

The datasheet distinguishes configurable digital measurement range from magnetic saturation. It lists saturation onset of 50 mT for standard IMC variants and 25 mT for the thin IMC variant. These are not interchangeable with usable output range; the actual part option, gain, and resolution must be recorded. [R3, Sections 3 and 13]

One three-axis reading cannot generally distinguish all magnet translations and rotations. The initial objective is repeatable class separation under constrained motions, not unrestricted 3D pose recovery.

### 2.4 Solist-AI relevance

ROHM describes ML63Q2557 within its AI-equipped MCU range. The official Solist-AI Sim supervised-learning guide includes a classification/grouping example with four target outputs. This supports investigating supervised classification, but is not a demonstration of tactile sensing. [R4][R5, Section M]

The paper's CNN will not be assumed to run unchanged on this MCU. Feature count, preprocessing, learning mode, memory use, and inference latency must be checked against the actual board, SDK, and model configuration. No firmware interface or resource budget is fixed here.

### 2.5 FDM implications

Prusa's flexible-filament guidance highlights moisture sensitivity, slow printing, and weak bridging/overhang performance. These are relevant process risks; they do not establish a working profile for the owner's printer. [R6]

This motivates an open structure, short spans, and a contact-face-down print orientation. A 45-degree feature is a design target, not a guarantee of support-free TPU printing.

## 3. Current v3 crossed-lattice geometry

The owner's revised requirement supersedes the split-carrier v1. The TPU lattice directly includes a cylindrical magnet cavity, 0.8 mm surrounding wall, floor, and roof. There is no separate holder, cover, or internal adhesive joint. The magnet is inserted during a pause and sealed by resumed printing.

All dimensions use print coordinates: contact face at Z = 0, mounting skin ending at Z = 10. The mounted frame is obtained by rotating about X: mechanical X = print X, mechanical Y = -print Y, mechanical Z = 10 - print Z.

| Feature | Implemented nominal dimensions |
| --- | --- |
| Envelope | 20 × 20 × 10 mm |
| Contact skin / pocket floor | Full 20 × 20 mm face, Z = 0–0.8 |
| Mounting skin | Full 20 × 20 mm face, Z = 9.2–10.0 |
| Magnet cavity | Diameter 3 × height 1.5 mm, Z = 0.8–2.3 |
| Radial pocket wall | 0.8 mm; outer diameter 4.6 mm |
| Pocket roof | 0.8 mm, Z = 2.3–3.1 |
| Pocket size compensation | Zero by default |
| Diamond outer diagonals | 8.8 × 8.8 mm; outer columns use inward half-diamonds |
| Cell centers in each bank | -8, 0, +8; Z = 4.8 |
| Rib banks | Orthogonal XZ and YZ; planes at -8, -4, 0, +4, +8 |
| Strut / rib depth | 0.8 mm normal to slope / 1.2 mm |
| Mid-height ties | Four 17.2 × 1.2 × 0.8 mm beams at X or Y = ±4; Z = 4.4–5.2 |

V2 used parallel planar ribs and exposed mounting feet. The owner identified missing depth connections, insufficient attachment area, and the missing second skin. V3 joins two perpendicular rib banks, adds interior ties, and closes both end faces. The core remains one connected solid even when both skins and pocket enclosure are excluded by a middle-height section. Ninety-degree CAD symmetry is checked, but physical stiffness and strength are not inferred from it. This is an independently authored crossed-rib structure, not a reproduction of the paper's 3D diamond unit cell.

The pocket wall moves with the contact skin and ends at Z = 3.1; it does not form a solid pillar to the backing. Junctions locally add material to the 0.8 mm radial wall. Mid-height ties and the second skin require TPU bridging trials; CAD connectivity alone does not prove printability.

The nominal pocket matches the magnet exactly, as requested. Printed shrinkage, extrusion width and magnet tolerances can change actual fit. Zero magnet motion has not been measured. A small configurable positive size compensation is available if required by a fit trial; none is applied in the default STL.

### Sensor and gripper interface

The magnet center is print Z = 1.55. The mounting skin fixes to a separate rigid nonmagnetic backing; the sensor board fixes behind it with the IC facing the magnet. A draft 34 × 34 mm backing with two PCB standoffs is provided separately as `mount-draft.stl`. It is not part of the single-piece TPU cushion.

The owner's board image specifies CJMCU-90393, 26.3 × 26 mm, with two holes along one edge. The selected [product listing](https://www.amazon.co.jp/dp/B0H8H693VS), inspected on 2026-09-18, states 20.4 × 20.4 mm instead. The supplied image takes precedence provisionally; exact dimensions require measurement. Draft hole centers at X = ±10, Y = +10 mm are image estimates. Board thickness 1.6 mm and active-plane offset 0.5 mm are assumed. The 3.2 mm base holes are trial M3 screw clearances, not measured PCB holes.

The draft uses a 3 mm backing and 3 mm standoffs, with flush countersunk heads on the cushion side and washers/nuts behind the board. It implies 13.95 mm nominal magnet-center-to-active-plane spacing before adhesive. The earlier 10 mm exploratory distance remains a cushion-only fixture setting, not a compatible position inside this base. Check signal/noise at the larger spacing and actual component, header, fastener and jaw clearances. See the [source guide](../src/tactile_fingertip/README.md) for assembly orientation and outstanding fit measurements. No robot-jaw adapter is included.

## 4. Pause-and-insert printing procedure

The default recipe uses a 0.4 mm nozzle and **uniform 0.1 mm layers including the first layer**, with the flat contact face down. This aligns the 0.8 mm floor and 1.5 mm cavity with actual layer boundaries. Do not uniformly scale or rotate the STL.

Pause after completing print Z = **2.3 mm**, before the first toolpath closing the pocket. For the specified schedule, 23 layers are complete and the next roof layer has its top at Z = **2.4 mm**. Inspect the sliced toolpaths because software layer numbering and pause placement semantics differ. A different first-layer height or adaptive layers invalidate this simple count.

At the pause, park the toolhead through the printer's supported procedure. Insert the magnet fully, record its axial polarity, and verify that its top is flush with or below the completed surface and remains seated. Resume to print the 0.8 mm roof and remaining lattice, ties and mounting skin. No post-print magnet-holder assembly is required.

STL stores geometry, not pauses. Set the pause in the actual slicer; no machine-specific pause G-code is supplied. Prusa documents layer pauses and embedding magnets as a general workflow, but this does not validate the owner's printer. [R8][R9]

Keep the enclosed pocket empty in the toolpath: disable internal supports and do not apply mesh repair that deletes the cavity surface. Fill the modeled TPU regions solid and confirm that thin ribs are preserved. The mid-height ties, mounting-skin bridges, small diamond closures and roof deposition over the inserted magnet require an actual print trial. Use the filament manufacturer's temperature/drying guidance, and verify the purchased magnet's suitability for the actual thermal process. Exact CAD fit is not proof of real fit or heat exposure tolerance.

Use a documented TPU/TPE grade, provisionally TPU 95A if the printer supports it. Around 15–25 mm/s is a starting trial, not a qualified profile. Record material, drying, speed, cooling, measured magnet dimensions, pause settings and print outcome. Check recovery, creep and retention before collecting training data.

## 5. Signal acquisition and classification experiment

Proposed chain: contact → lattice/contact-skin deformation → magnet translation/rotation → MLX90393 Bx/By/Bz → timestamped acquisition → baseline subtraction and features → Solist-AI classification.

First prove that controlled press and lateral motion produce stable, separable signals. Then evaluate Slip. Do not label all vibration as slipping: motor activity, tapping, impacts, and release can also produce transients.

| Label | Operational definition for data collection |
| --- | --- |
| No Contact | Unloaded baseline; record both quiet and motor-active conditions separately |
| Normal Press | Controlled inward motion, without intended lateral sliding |
| Left Shear | -X tangential deformation under a defined normal preload, before surface sliding |
| Right Shear | +X tangential deformation under the same preload, before surface sliding |
| Slip | Observed object motion relative to the contact skin during maintained contact |

Labels are experiment definitions, not mutually exclusive physical phenomena. Shear can transition into slip. Use video or a marked object to establish slip onset independently of the magnetic waveform; exclude ambiguous transition windows from the first training set.

Target approximately 100–200 complete XYZ samples/s for exploratory acquisition if the chosen sensor settings, bus, and logging sustain it. Record actual timestamps, dropped samples, gain, filtering, and oversampling. Useful slip frequencies may lie outside this bandwidth, so success is not guaranteed. Static press/shear tests can proceed at a lower verified rate.

Start with baseline-subtracted means, slopes, ranges, and short-window variation on each axis. A raw 50-sample, three-axis window would contain 150 values, but that count is only an example; MCU support and model resource use remain unverified. Fix normalization from training data and avoid baseline adaptation during maintained contact.

Collect separate repetitions at several indentation levels and lateral offsets. Proposed initial bench increments are 0.5 and 1.0 mm compression, then small lateral excursions under preload; stop before damage or bottoming. These are fixture exploration targets, not robot motion limits. Use a load cell or scale if reporting force, because displacement is not a calibrated force measurement.

Split training and test sets by complete trial/session and, later, printed specimen and object. Do not randomly split overlapping windows from the same gesture across both sets. Report a confusion matrix, per-class recall, false slip events, and latency alongside the material, fixture, and acquisition conditions.

## 6. Implementation and deliverables

The source is under `src/tactile_fingertip/`: `config.py` contains dimensions, `geometry.py` builds the single solid, `generate.py` exports STL, `validation.py` checks geometry, and `preview.py` renders a full view and a true CAD section. See [generation instructions](../src/tactile_fingertip/README.md).

Current outputs are in [tactile-fingertip](../3d-models/tactile-fingertip/README.md). Print `fingertip.stl` in TPU. `mount-draft.stl` is an optional rigid fit prototype with unverified board dimensions. The magnet shown in the preview is a reference and is not included in that print file. Exports are overwritten in place; old models and version subdirectories are not retained.

CadQuery provides the editable parametric source and STL export. [R7] No robot-control software or reference jaw geometry is changed.

## 7. Verification and pending physical work

Ten fingertip regression tests passed: exact pocket boundaries, radial wall, floor/roof, single-solid topology, pause opening/insertion access, both skins, interior connectivity, rotational symmetry, connectivity to the bed at all 100 layer boundaries, invalid settings, 0.5/1.0 mm wall variants and STL export checks. STL volume and cavity size agree with CAD.

Three additional tests passed for the draft mount: plate/boss and screw access, invalid parameters, and STL export. These are geometry checks only.

A closed pocket creates an exterior surface and an inward-facing cavity surface in STL. These two closed surface shells bound **one material solid**, not two disconnected TPU components. Validation checks both their orientation and volumes. Do not delete the inner shell during slicer repair.

| Gate | Status |
| --- | --- |
| CAD bounds, one solid, cavity/walls, insertion access | Passed for v3 |
| STL closure/orientation/volumes | Passed |
| Actual slicer toolpaths and printer pause/resume | Pending |
| Printed fit, enclosure integrity and magnet retention | Pending |
| Stiffness, creep, recovery and fatigue | Pending |
| Sensor range/noise and classification | Pending |
| PCB holder, fixture and installed jaw fit | Draft base exported; fit and jaw attachment pending |

For repeatability, precondition and then record at least 20 controlled loading/unloading cycles at a documented displacement, rate and dwell time. Compare signal spread and baseline recovery with class separation. Establish acceptance thresholds from measured noise and application needs; no force range, print success or Slip accuracy is currently certified.

Open inputs remain the exact printer/filament, magnet tolerances and grade, MLX90393 board/part option, supply/logic compatibility, fixture/jaw geometry, and acceptable stiffness/travel. These do not prevent the standalone coupon; they prevent final installed-performance claims.

## 8. Public references

Accessed/reviewed on 2026-09-18. Sources describe prior work or component behavior; dimensions in Section 3 are this project's prototype choices.

- **[R1]** Yunseong Bang, Joowon Park, Suan Sim, Youngjun Ryu, Sukho Park, and Kyungseo Park. *Magnet-Based Soft Robotic Skin Using a 3D-Printed Multi-Lattice Structure and CNN-Based Tactile Super-Resolution*. arXiv:2605.28352, 2026. [Record](https://arxiv.org/abs/2605.28352), [reviewed paper](https://arxiv.org/pdf/2605.28352). Relevant: Sections II–IV; not a source of this prototype's geometry.
- **[R2]** Yunseong Bang. *Magnet-Based Soft Robotic Skin with Multi-Lattice Structure for Tactile Super-Resolution via Sim-to-Real Transfer*. DGIST master's thesis, 2026. [Repository record](https://scholar.dgist.ac.kr/handle/20.500.11750/60784). Metadata/contents only; full thesis not reviewed.
- **[R3]** Melexis. *MLX90393 Datasheet*, revision 012, November 18, 2025. [Official download page](https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90393), [reviewed PDF](https://media.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf). Relevant: part options, timing, magnetic range, and configuration.
- **[R4]** ROHM. *ROHM Develops Breakthrough AI-Equipped MCUs*, June 4, 2025. [Official announcement](https://www.rohm.com/news-detail?defaultGroupId=false&news-title=2025-06-04_news_micon). Product-family context; no tactile performance claim.
- **[R5]** ROHM. *Solist-AI Sim Supervised Learning Quick Start Guide*, document CTD_MBDG_68_029. [Official PDF](https://fscdn.rohm.com/lapis/en/products/databook/applinote/ic/micon/Solist-AI_Sim_SupervisedLearning_qs-e.pdf). Section M, pp. 72–77: classification/grouping example.
- **[R6]** Prusa Research. *Flexible materials*. [Official printing guidance](https://help.prusa3d.com/article/flexible-materials_2057). General flexible-filament process limitations; printer-specific settings require separate verification.
- **[R7]** CadQuery project. [Examples](https://cadquery.readthedocs.io/en/latest/examples.html) and [Importing and Exporting Files](https://cadquery.readthedocs.io/en/latest/importexport.html). Official documentation consulted via Context7 for the proposed modeling/export workflow.

- **[R8]** Prusa Research. [Insert pause or custom G-code at layer](https://help.prusa3d.com/article/insert-pause-or-custom-g-code-at-layer_120490). Slicer pause workflow; printer-specific compatibility must be verified.
- **[R9]** Prusa Research. [The Surprising Practical Uses of Color Change in PrusaSlicer](https://blog.prusa3d.com/practical-uses-of-color-change-in-prusaslicer_59563/). Examples of embedding magnets during a print pause.
