<!-- docs/spec/operating-modes.md: Record the owner-decided operating modes, their triggers, and what each needs before it can run on the robot. -->
# Operating Modes

Written: 2026-09-24. Status: owner decisions on the mode structure, KachiButton mapping, arm engagement, Auto Catch alignment, Auto Release motion, voice input, emergency stop, and home-position handling, recorded from the design discussion on that date; the specification is considered settled for Phase 1. Manual Mode (driving, leader-arm puppeteering with an egg caught, `Go Go!` mode switching, slow arm re-synchronization) was owner-tested on the robot on 2026-09-24; Phase 1 is complete. Auto Catch alignment and Auto Release (recorded motion into the pink basket) were owner-tested on the robot on 2026-09-24 and 2026-09-25. The stop unit (`STOP`, `OFF`, `MODE`) was owner-tested on the robot on 2026-09-26. Everything else is design intent until the sections below say otherwise.

This document supersedes the three-mode table in [concept.md](../proposals/concept.md) (Drive, Puppet, Autonomous). Drive Mode and Puppet Mode run at the same time as Manual Mode; the autonomous experience is split into Auto Catch, Auto Release, and Full Self-Catching.

## 1. Modes (owner decisions, 2026-09-24)

| Mode | What the attendee does | What the robot does | Depends on |
| --- | --- | --- | --- |
| **Manual Mode** | Drives with the dino-controller and/or moves the leader arm by hand. One person can do either; two people can do both at once | Base follows the controller (see [src/robot/README.md](../../src/robot/README.md)); the dinosaur arm follows the leader arm | Controller and leader arm connected |
| **Auto Catch** (inside Manual Mode) | Presses `Hi!` when the dinosaur roughly faces an egg | Finds the egg in the front camera, drives the base with image-based control until the egg sits at the predefined position in the image, then runs the wrist-camera pick policy and grasps the egg with the mouth. If no egg is visible or it is too small (too far), an error is shown and nothing moves | Front-camera egg detector, base alignment controller, trained `pick_egg` policy |
| **Auto Release** (inside Manual Mode) | Presses `Thx` when the pink basket is in front of the dinosaur | Checks that the pink basket is in view, moves the arm to the base pose (neck folded, gripper angle unchanged so the egg stays held), then plays a recorded release motion into the basket | Front-camera basket detector, recorded release motion |
| **Full Self-Catching (FSC)** | Says, for example, "catch the green egg", then confirms or rejects the highlighted egg by voice | Gemini Robotics ER 2 proposes candidates, the signboard draws a rectangle around one, the attendee confirms by voice, then the base drives until the egg is in the front camera, runs the same alignment and catch as Auto Catch, drives back to the pink basket using its color as a beacon, and runs the same release as Auto Release | Overhead camera, Gemini ER 2, speech input, navigation, Auto Catch and Auto Release |

Auto Catch and Auto Release exist only as attendee-triggered actions in Manual Mode. FSC contains the same two procedures and runs them automatically, so they are built first and shared. FSC is a demonstration run by staff, not the default attendee experience; how long one run takes is measured once it works. The Gemini, navigation, and speech parts are designed in [dino-egg-catch-challenge-gemini-integration.md](../proposals/dino-egg-catch-challenge-gemini-integration.md).

### Arm poses

The arm has two recorded poses (owner decision, 2026-09-25). The **release pose** ("home"): joints folded with the head pointing **up** (a downward head hit the basket during release); this is the pose for driving with an egg and the start of Auto Release. The **catch pose**: head pointing **down** so the wrist camera sees the egg at the aligned position; the pick policy starts here with the egg already in view, and the app moves between the two poses with the slow scripted approach. Both are recorded from the leader arm with staff keys (`b` release/home, `k` catch) into `data/arm/`. In this pose the dinosaur can hold an egg and be driven around without anyone touching the leader arm, which is what makes single-player Manual Mode possible. The home pose is recorded once on the robot from the signboard keyboard (`b` key, staff only) into `data/arm/home_pose.json`; it is also the base pose that Auto Release starts from. The release motion is recorded the same way (`r` key toggles recording) into `data/arm/release_motion.json`. Staff keys never use a letter that a KachiButton phrase types (so not `h`, which "Thx" and "Hi!" contain).

## 2. KachiButton controls (owner decisions, 2026-09-24)

The KachiButton is a three-key USB keyboard keychain (companion repository `CreatorKanata/kachi-button`). With its default firmware each press types one plain ASCII phrase, with no Enter and no repeat. Two units are connected to the exhibit Mac:

| Unit | Physical key | Typed phrase | Action |
| --- | --- | --- | --- |
| Mode unit | Top | `Go Go!` | Toggle between Manual Mode and FSC |
| Mode unit | Lower left | `Hi!` | In Manual Mode: start Auto Catch. In FSC: first press starts voice input, second press ends it |
| Mode unit | Lower right | `Thx` | In Manual Mode: start Auto Release. In FSC: no action (owner decision) |
| Stop unit | Top | `STOP` | Emergency stop, in every mode |
| Stop unit | Lower left | `OFF` | Arm torque off (staff), in every mode; also a stop |
| Stop unit | Lower right | `MODE` | Return to Manual Mode, in every mode; the only way out of a stop |

Stop unit keys and their physical positions (top `STOP`, lower left `OFF`, lower right `MODE`): owner decision, confirmed 2026-09-25. The unit types the phrases in capitals exactly as shown (owner-confirmed 2026-09-26).

The application watches keyboard input for these phrases. The signboard window has keyboard focus for the whole exhibit (owner decision: this is the operating assumption, so no global keyboard hook is planned). Typed characters are matched only as complete phrases; input with more than 1 s between characters is discarded.

Rules that apply to every switch:

- Switching modes first sends zero base velocities and holds the arm where it is. No mode starts with motion.
- Auto Catch and Auto Release are one-shot actions that return to Manual Mode when they finish, fail their precondition, or are stopped.
- A press while an automatic action is running is ignored; `STOP`, `OFF`, and `MODE` always work.
- The signboard shows the current mode in large text (Manual or FSC first; richer artwork later), the running action, and for a rejected Auto Catch or Auto Release the reason ("No egg in view", "Egg too far", "Egg too close", "Egg lost", "Could not align", "Basket not in view", "Basket too far", "Basket lost", "Home pose not recorded", "Release motion not recorded", "Arm did not reach home", "Catch pose not recorded", "Egg not in wrist view", "Arm did not reach the catch pose", "Arm did not reach the release pose") for a few seconds in the warning color; informational notices such as "Catch: policy not available yet" and "Ready" use the normal color. A failure warning is not replaced by "Ready" when the arm returns to the release pose. There is never a dialog to dismiss: attendees have no cursor.

### Emergency stop behavior

`STOP` (or ESC / closing the signboard, or Ctrl+C) is a full stop and full reset (owner decision): it zeroes the base, cancels any automatic action, discards pending encoder rotation, clears voice input, and holds the arm at its current pose with torque kept on. After `STOP` the application is latched in a stopped state: the base stays at zero and the arm stays held, `Hi!`, `Thx`, and `Go Go!` are ignored, and the signboard shows "STOPPED. Press MODE to resume" until the stop unit's `MODE` is pressed (owner decision, 2026-09-25; before that `Go Go!` resumed). That press returns to Manual Mode (never FSC) with arm following disengaged; following then resumes through the slow engagement below. `MODE` also works without a stop: it cancels a running action and returns to Manual Mode. Nothing moves again on its own after a stop (owner decision, 2026-09-24).

`STOP` keeps the arm torque on: the SO-ARM101 has no brakes, so a limp arm would fall under gravity, drop a held egg, and could hit the field or a hand. `OFF` is the explicit release (below); `STOP` never changes the torque, so a `STOP` after `OFF` leaves it off. This stop travels through the signboard process and the control loop, so it is a software stop measured in tens of milliseconds; the Pi host's 500 ms command watchdog remains the last line.

### Arm torque off (owner decision, 2026-09-25)

`OFF` is for staff: moving the arm by hand or clearing a jam. It is a stop as above (base zeroed, action cancelled, latched) and also releases the arm torque, so the arm goes limp and falls under gravity: hold it or keep it close to the table before pressing. The base stays at zero with its wheel torque on. The signboard shows "TORQUE OFF", "Press MODE to resume", and "arm torque off". `MODE` re-enables the torque at the arm's present position (the application first re-reads that pose, since the arm may have been moved), then the slow engagement re-synchronizes it to the leader. The application sends the release as an extra `arm_torque` action key; only the fork's updated host on the Pi reads it (an older host ignores it and the arm stays stiff), so the Pi must run a fork commit that reads `arm_torque`. Owner-tested on the robot on 2026-09-26: `OFF` released the arm and `MODE` re-enabled it and returned to Manual Mode.

## 3. Manual Mode details

- Base: exactly the Drive Mode behavior already implemented (joystick translation, encoder rotation, fixed speed level, zero velocities on input loss).
- Arm: follows the leader arm (`so100_leader`, id `dino_leader_arm`) joint for joint, including the gripper (the mouth). Leaving the leader arm resting at its home position keeps the dinosaur arm at home; this is the operating rule (owner decision), and there is no separate "go home" button.
- Engagement (owner decision): when Manual Mode starts, or after a stop, the follower moves slowly toward the leader's pose at `ARM_ENGAGE_SPEED_DEG_S` per joint. Real-time following begins only once every joint is within `ARM_ENGAGE_TOLERANCE_DEG` of the leader. The signboard shows "arm syncing" until then. This removes the jump a mismatched leader would otherwise cause.
- If the leader arm cannot be read (disconnected, bus error), the follower holds its last commanded pose and the signboard shows the fault; the base keeps working.
- The shaft button on the dino-controller has no role (`SHAFT_BUTTON_ROLE = "none"`): Catch moved to `Hi!`, stop is the second KachiButton. It stays wired and reported for a later use.

## 4. Auto Catch and Auto Release details

### Auto Catch

1. Precondition, evaluated on the front camera at the press: an egg is detected and its bounding-box height, as a fraction of the frame height, lies between `AUTO_CATCH_MIN_EGG_H` (too far) and `AUTO_CATCH_MAX_EGG_H` (too close). Otherwise "No egg in view", "Egg too far", or "Egg too close" and nothing moves. The best position is stored as the normalized bbox center and height (`ALIGN_TARGET_*`) measured from a reference frame captured on the robot with the egg placed where the owner wants it; the signboard draws that position as an egg-shaped outline at all times in Manual Mode so it also guides manual driving (owner suggestion, 2026-09-24).
2. Start pose (owner decision, 2026-09-25): if the arm is not near the release pose when `Hi!` is pressed (the leader may have left it anywhere), it first moves to the release pose so that driving and the later move to the catch pose follow the one known-safe path; otherwise this step is skipped. Scripted pose moves interpolate all joints so they arrive together.
3. Alignment (owner decision): the front camera gives the egg's position relative to the body. An image-based controller drives the base until the egg reaches the predefined target position and size in the front image, the one the pick policy was trained from. This step is what raises the catch success rate; the pick policy never has to compensate for base placement.
4. Catch pose: the arm moves slowly to the recorded catch pose (head down). The wrist camera is mounted at an angle, so the egg may sit at the edge of the wrist view or, when far but aligned, outside it; this is accepted (owner decision, 2026-09-25) because the policy also uses the joint state and the demonstrations cover both cases. A wrist-view egg check exists as a diagnostic and stays disabled as a gate.
5. Catch: the pick policy runs on the wrist camera only (owner decision, consistent with the golf-ball result), from the catch pose until the egg is held. Success is checked before the signboard says "caught".
6. The arm moves slowly to the release pose (head up) with the egg held; driving and Auto Release continue from there.

The pick policy is trained in the LeRobot fork (ACT first, SmolVLA as a comparison) and consumed here through a policy runner that can be interrupted. Demonstrations are recorded with this repository's recorder (`robot.recording.record_pick_egg`), which drives the fork's `record_loop` with the leader arm and keyboard teleoperators (owner decision, 2026-09-25; the `lerobot-record` CLI cannot drive LeKiwi with a lone arm teleoperator): the robot is parked on a floor mark, eggs are placed at a marked best position with small deliberate offsets inside the alignment tolerance, and each episode runs from the catch pose (head down) through the grasp and back to the catch pose with the egg held, then one second of stillness; the application raises the arm afterwards. The application reproduces that start state at demo time through alignment and the catch pose.

### Auto Release

1. Precondition: the pink basket is detected in the front camera and is wide enough (`AUTO_RELEASE_MIN_BASKET_W`, normalized width). Otherwise "Basket not in view" or "Basket too far".
2. Alignment (owner decision, 2026-09-24): the base aligns to the basket with the same image-based controller as Auto Catch (basket center x and apparent width, reference from the owner's captures at the release position), so the recorded motion always starts from the same relative pose.
3. The arm moves to the home/base pose (neck folded) while keeping the gripper angle unchanged so the egg stays held.
4. A recorded release motion (a fixed joint trajectory, not a learned policy) delivers the egg into the basket and opens the mouth, then returns to home.

Egg colors (owner decision, 2026-09-24): white eggs with **green, red, or yellow** spots. Blue spots were dropped that day because on the robot they measured the same hue, saturation, and value as the blue tarp; the yellow egg is painted (its glossy spots read orange-ish, hue 5-18, on the front camera; verified 2026-09-25). The detector is local (no network call), anchored on the spots found as edge ellipses surrounded by white egg body, classified by spot color, with the pink basket excluded by color. Thresholds are calibrated from captures on the robot and stored in the vision config.

## 5. FSC details that follow from the above

- Outbound: Gemini ER 2 on the overhead image selects the target; the base drives toward it until the egg appears in the front camera; from there Auto Catch's alignment and catch take over. The overhead camera therefore only has to get the robot close, which relaxes its calibration.
- Return: the robot turns until the pink basket is in the front camera, drives until it reaches the Auto Release size, and runs Auto Release.
- Voice (owner decision, 2026-09-24): `Hi!` toggles listening (push-to-talk by toggle) so the noisy venue does not trigger false input. The same rule applies to the answer to "is this the egg?": the attendee presses `Hi!` to start listening, answers by voice, and presses `Hi!` again to end. Listening never starts by itself. The signboard shows a clear status for each step: "Listening...", "Thinking...", "Is this the egg?" with the rectangle, "Confirmed", and "Driving to the egg" and so on, so nobody wonders whether the robot is waiting for them. A keyboard fallback for staff is kept.
- Target identity: the egg the attendee confirmed on the overhead image must be the egg the front camera aligns to. The overhead tracker's target ID is checked against the front-camera detection before Auto Catch's alignment starts; when the match is uncertain (for example two eggs of the same color close together), FSC asks again instead of guessing.
- Return fallback: if the pink basket cannot be found from the front camera (occluded by people), the robot uses the overhead position estimate to drive near the basket and then searches again.
- To verify on the robot before FSC work: the front camera's view with the neck folded and an egg held (the head must not block it), and the speech stack (speech-to-text, Gemini ER 2 model, text-to-speech) is chosen in Phase 4 planning.

## 6. Implementation order

| Phase | Scope | Done when |
| --- | --- | --- |
| 1 | Mode manager, KachiButton phrase detection through the signboard (`Go Go!`, `Hi!`, `Thx`, `STOP`; since 2026-09-25 also `OFF`, `MODE`), Manual Mode with leader arm and slow engagement, mode text on the signboard. Auto Catch, Auto Release, and FSC show "not available yet" and do not move the robot | Owner drives and puppets, alone and with a second person; the four phrases change the display and `Stop` zeroes the base |
| 2 | Front-camera egg and basket detectors with venue calibration; Auto Catch precondition and base alignment; Auto Release precondition, home pose, and recorded release motion | Alignment ends with the egg at the target image position; release delivers a held egg into the basket |
| 3 | Catch pose and wrist-view check in the Auto Catch flow; episode recording from the catch pose; ACT training in the fork; policy runner for the trained `pick_egg` checkpoint (wrist camera), success check, one automatic retry | Auto Catch succeeds from a `Hi!` press |
| 4 | FSC: overhead camera, Gemini ER 2 target selection with rectangle confirmation, voice in and out, drive to the egg, return by pink beacon | End-to-end run from a spoken request to a released egg; duration measured |

## 7. Open points

- Home pose values and the release motion: recorded on the robot in Phase 2.
- Engagement speed and tolerance: tuned on the robot in Phase 1.
- Speech device, speech-to-text engine, and TTS for FSC progress (Phase 4).
