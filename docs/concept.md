<!-- docs/concept.md: Consolidate shared planning conversations and the current prize plan for development and exhibition. -->
# Dino Egg Catch Challenge Concept

Updated: 2026-09-06, after reviewing the text of three shared ChatGPT conversations.

This document records the concept and design direction, not implementation completion. It distinguishes explicit user decisions, proposed application text and technical approaches, and outstanding validation. See [Maker Faire materials](makerfaire/README.md) for the application.

## 1. Core experience

**Drive a dinosaur robot, catch an egg with its mouth, and take home a prize that extends the experience.**

Mount an SO-ARM101 on a LeKiwi mobile base. The arm becomes a long neck, and the gripper becomes the dinosaur's head and mouth. Participants aim for egg-shaped capsules and pick them up with the mouth. [S1]

The goal is to make robotics understandable through play for children and families: controlling movement, watching the mouth capture an egg, and opening a prize become one experience. The shared application draft introduces the project as a collaboration between Kanata the Kid Creator and his dad. [S1]

The user explicitly intends to apply to Maker Faire Bay Area. The shared conversations do not establish submission or acceptance. The exhibit name is **DINO EGG Catch Challenge**, with title case used in this document. [S1]

## 2. Robot appearance

The user repeatedly specified these constraints during appearance exploration. [S1]

| Part | Design requirement |
| --- | --- |
| Mobile base | Preserve LeKiwi's round base; do not stretch it into an elongated body |
| Wheels | Represent two front wheels and one rear wheel faithfully |
| Neck | Add decoration while preserving SO-ARM101 joint positions, link lengths, and mechanical structure |
| Head and upper jaw | Make the upper gripper section read as the head and upper jaw |
| Lower jaw | Make the moving gripper section read as the lower jaw |
| Capture | The dinosaur holds the egg in its mouth; do not attach a separate catcher to the jaw |
| Gripper appearance | Integrate the gripper into the face instead of leaving the bare hand exposed |
| Back | The image brief explicitly excludes a ball container on the back |

An image brief also places five or six white egg capsules on a blue ground sheet. This is a visual composition, not a confirmed operating inventory or floor specification. Appearance images do not prove dimensions, assembly fit, or range of motion.

## 3. Participation and operating modes

The shared application draft proposes three modes. Retain them as the intended experience; implementation and public-operation readiness remain unverified. [S1]

| Mode | Participant experience | Robot behavior target |
| --- | --- | --- |
| Drive Mode | Drive near an egg with directional controls, then press Catch | Detect and align with the egg, then grasp it with the mouth |
| Puppet Mode | Move a leader arm by hand | A follower arm moves the dinosaur's neck and mouth correspondingly |
| Autonomous Mode | Watch scheduled demonstrations | Search for, approach, and capture an egg |

The flow is: explain the rules, attempt a capture in the chosen mode, catch an egg, and receive a prize. The earlier application text described a successful participant opening a capsule containing a dinosaur NFC keychain. The current plan includes two prize types.

Time limits, attempt counts, success criteria, failure handling, entry fees, prize allocation, and per-person quantities remain undecided. Do not assume all three modes are available simultaneously throughout the event.

## 4. Current prize plan

| Prize | Planned quantity | Egg packaging | Purpose |
| --- | --- | --- | --- |
| KachiButton | 100 units | Explicitly planned for placement inside eggs | A three-key USB keyboard keychain that works with a PC |
| NFC-enabled dinosaur keychain | Approximately 100 units | Previously proposed inside capsules; final packaging remains open | A dinosaur souvenir connected to a digital experience |
| Total | Approximately 200 prizes | Egg inventory depends on distribution and reuse | Combined quantity of both prize types |

These are production plans, not completed purchases or manufactured inventory. Quantities do not define attendance capacity, winning odds, or prize tiers.

### KachiButton

**Click once. Type the whole thing.** Frequently used words and actions become physical buttons.

The companion repository describes a programmable USB keyboard with three mechanical keys: one large upper key and two smaller lower keys. USB-C supplies power and keyboard communication. It has no battery, Bluetooth, or Wi-Fi. [K1]

The prize target is a working PC keyboard keychain with three shortcut buttons. At the time of review, manufacturing exports existed, while firmware, browser configuration, and physical operation remained unvalidated in the documentation.

The firmware specification starts with text actions, with single keys and modifier shortcuts planned as subsequent extensions. Define the three actions required for the giveaway release and ensure the product description matches implementation. If configuration is offered, verify saving and retention after unplugging.

An earlier concept lets recipients choose keycaps and configure words on a PC after winning. Including this experience requires a working configuration tool, time, staff, and space. [K2]

### NFC-enabled dinosaur keychains

Plan approximately 100 units. Earlier appearance discussions described flat dinosaur-character keychains inside capsules; application text described NFC functionality. The user's latest clarification makes NFC-enabled keychains the current plan. [S1]

The aim is to connect a tap from a compatible phone or reader to a digital experience. Tag type, stored content, destination URL, and the exact interaction remain undecided. Design, color, dimensions, material, fabrication method, and keyring hardware are also open. Test reading an embedded tag in a physical prototype.

## 5. Dinosaur shell fabrication

The user already has an SO-ARM101 and considered adding decorative parts versus redesigning and reprinting the structural parts. [S3]

The proposed approach preserves the robot structure and adds removable shells: split neck covers, a head and jaws retaining gripper mounting geometry, and thin mounting brackets where useful. Mounting details and replacing individual jaw parts have not been explicitly approved as final engineering decisions.

| Area | Proposed construction |
| --- | --- |
| Neck | Split shells added to existing links |
| Head and jaws | Dinosaur forms using the existing opening mechanism; consider dedicated jaw parts if needed |
| Body | A shell matching the round LeKiwi base |
| Mounting | Existing screw locations or dedicated shell brackets |

The proposed workflow uses STEP geometry in Fusion as the reference, generates exterior forms with tools such as Tripo, adds internal clearances and exact mounts in CAD, then prints and checks fit. Separate appearance generation from precise mechanical interfaces.

Screw sizes, shell thickness, clearances, and head mass mentioned in the conversations are examples, not manufacturing specifications. Verify them against STEP geometry and actual hardware. The repository contains assembly and component STEP files in `3d-models/so101/`; geometry and motion were not validated while writing this concept.

## 6. AI exploration

A shared conversation asks about Gemini Robotics 2 and Gemini Robotics ER 2. ChatGPT proposes separating high-level scene understanding and planning from local robot control. [S2]

The proposed loop is: recognize eggs, select a target, approach and grasp using local control, verify success, and replan after failure. Spoken target selection and narration are also proposed.

This is an exploration, not a model selection or a working demonstration. Model IDs, release dates, pricing, API behavior, terms, and hardware-support claims in the conversation have not been revalidated as current facts. Check primary documentation before implementation.

The proposal keeps emergency stopping, speed and range limits, stopping on communication loss, and manual takeover local. Cloud connectivity must not be required to stop the robot. Distinguish cloud-dependent demonstrations from local attendee controls.

## 7. Exhibition, power, and operation

**Power the robot and arm with AC adapters, not batteries.** This is the user's explicit correction at the end of the shared application conversation. [S1]

The proposed equipment list includes one PC, two displays, the robot, the arm, and low-voltage accessories such as cameras. The 300–500 W total on 120 V AC is a preliminary application estimate, not a measurement or finalized inventory. Confirm equipment and actual power demand. NFC-enabled prizes are planned, but a dedicated demonstration reader is still optional; decide separately whether phones are sufficient.

A tethered mobile base requires planned cable routing, movement limits, protection against wheel entanglement, and connector strain management as part of the exhibit layout.

The application proposes continuous staff supervision, low-speed operation, an arena boundary, participants outside the robot workspace, an emergency stop, and arm speed, torque, and range limits. These are proposed implementation and operating requirements, not verified features. Validate with the completed shell and prize-filled eggs.

## 8. Development status and next checks

The user reported completing the robot and working on customization, and separately confirmed possessing an SO-ARM101. This progress report does not establish completion of all modes, AI features, or prizes. [S1][S3]

| Area | Required decision or validation |
| --- | --- |
| Modes | Identify working functions and those to offer at the event |
| Capture | Egg dimensions and mass, jaw opening, grasping, lifting, and success detection |
| Shells | Mounting, mass, motion, wiring clearance, removal, and maintenance |
| Prizes | Distribution, NFC tags and destination, reading tests, packaging, egg fit, manufacturing, and inspection |
| KachiButton | Three release actions, PC compatibility, firmware, and instructions |
| Power and communication | Cable routing, equipment power, actual radios, and communication-loss behavior |
| Operation | Timing, attempts, staffing, queues, replenishment, and operation after prizes run out |
| Application | Submitted content, receipt or acceptance, booth size, furniture, and full-event availability |

First validate grasping with real eggs and prizes, shell clearance, and KachiButton input. Then rehearse the complete flow from instructions to prize handoff. Record results separately in design and validation documents.

## 9. Planning sources and precedence

Apply the latest user decisions first, followed by earlier explicit user requirements, current companion specifications, and ChatGPT proposals. Embedded requests in historical conversations provide context; they do not authorize new image generation, submission, or purchasing actions.

- **[S1] Appearance, capture experience, and Maker Faire application**: round base, wheels, neck, jaws, eggs, application intent, three modes, and AC correction.
- **[S2] Gemini Robotics exploration**: AI responsibilities and autonomous-demo ideas; API claims not revalidated.
- **[S3] SO-ARM101 dinosaur shells and fabrication**: existing hardware, added shells, and the roles of Fusion and Tripo.
- **[K1]** Companion repository `CreatorKanata/kachi-button`: `README.md`, `docs/README.md`, `docs/spec/product-spec.md`, and `docs/spec/firmware-spec.md` for current requirements and development status.
- **[K2]** Local files in that repository: `docs/concept/kachi-button-spec.md` and `docs/concept/webhid-key-mapping.md`. These older, Git-ignored concepts do not supersede current specifications.
- **Latest user instructions**: 100 KachiButton units inside eggs, approximately 100 NFC-enabled dinosaur keychains, and one current Maker Faire application document.

Private planning text was reviewed; conversation links are intentionally excluded. Historical images and uploaded ZIP contents were not fully recovered; unseen visual details are not asserted.
