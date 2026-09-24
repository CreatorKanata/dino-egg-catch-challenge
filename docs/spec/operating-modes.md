<!-- docs/spec/operating-modes.md: Record the owner-decided operating modes, their triggers, and what each needs before it can run on the robot. -->
# Operating Modes

Written: 2026-09-24. Status: owner decisions on the mode structure, KachiButton mapping, arm engagement, Auto Catch alignment, Auto Release motion, voice input, emergency stop, and home-position handling, recorded from the design discussion on that date; the specification is considered settled for Phase 1. Manual Mode (driving, leader-arm puppeteering with an egg caught, `Go Go!` mode switching, slow arm re-synchronization) was owner-tested on the robot on 2026-09-24; Phase 1 is complete. Everything else is design intent until the sections below say otherwise.

This document supersedes the three-mode table in [concept.md](../proposals/concept.md) (Drive, Puppet, Autonomous). Drive Mode and Puppet Mode run at the same time as Manual Mode; the autonomous experience is split into Auto Catch, Auto Release, and Full Self-Catching.

## 1. Modes (owner decisions, 2026-09-24)

| Mode | What the attendee does | What the robot does | Depends on |
| --- | --- | --- | --- |
| **Manual Mode** | Drives with the dino-controller and/or moves the leader arm by hand. One person can do either; two people can do both at once | Base follows the controller (see [src/robot/README.md](../../src/robot/README.md)); the dinosaur arm follows the leader arm | Controller and leader arm connected |
| **Auto Catch** (inside Manual Mode) | Presses `Hi!` when the dinosaur roughly faces an egg | Finds the egg in the front camera, drives the base with image-based control until the egg sits at the predefined position in the image, then runs the wrist-camera pick policy and grasps the egg with the mouth. If no egg is visible or it is too small (too far), an error is shown and nothing moves | Front-camera egg detector, base alignment controller, trained `pick_egg` policy |
| **Auto Release** (inside Manual Mode) | Presses `Thx` when the pink basket is in front of the dinosaur | Checks that the pink basket is in view, moves the arm to the base pose (neck folded, gripper angle unchanged so the egg stays held), then plays a recorded release motion into the basket | Front-camera basket detector, recorded release motion |
| **Full Self-Catching (FSC)** | Says, for example, "catch the green egg", then confirms or rejects the highlighted egg by voice | Gemini Robotics ER 2 proposes candidates, the signboard draws a rectangle around one, the attendee confirms by voice, then the base drives until the egg is in the front camera, runs the same alignment and catch as Auto Catch, drives back to the pink basket using its color as a beacon, and runs the same release as Auto Release | Overhead camera, Gemini ER 2, speech input, navigation, Auto Catch and Auto Release |

Auto Catch and Auto Release exist only as attendee-triggered actions in Manual Mode. FSC contains the same two procedures and runs them automatically, so they are built first and shared. FSC is a demonstration run by staff, not the default attendee experience; how long one run takes is measured once it works. The Gemini, navigation, and speech parts are designed in [dino-egg-catch-challenge-gemini-integration.md](../proposals/dino-egg-catch-challenge-gemini-integration.md).

### Arm home position

The arm has a home position: joints folded, mouth pointing down. In this pose the dinosaur can hold an egg and be driven around without anyone touching the leader arm, which is what makes single-player Manual Mode possible. The home pose is measured once on the robot and stored in `config.py`; it is also the base pose that Auto Release starts from.

## 2. KachiButton controls (owner decisions, 2026-09-24)

The KachiButton is a three-key USB keyboard keychain (companion repository `CreatorKanata/kachi-button`). With its default firmware each press types one plain ASCII phrase, with no Enter and no repeat. Two units are connected to the exhibit Mac:

| Unit | Physical key | Typed phrase | Action |
| --- | --- | --- | --- |
| Mode unit | Top | `Go Go!` | Toggle between Manual Mode and FSC |
| Mode unit | Lower left | `Hi!` | In Manual Mode: start Auto Catch. In FSC: first press starts voice input, second press ends it |
| Mode unit | Lower right | `Thx` | In Manual Mode: start Auto Release. In FSC: no action (owner decision) |
| Stop unit | Top (reconfigured) | `Stop` | Emergency stop, in every mode |

The application watches keyboard input for these phrases. The signboard window has keyboard focus for the whole exhibit (owner decision: this is the operating assumption, so no global keyboard hook is planned). Typed characters are matched only as complete phrases; input with more than 1 s between characters is discarded.

Rules that apply to every switch:

- Switching modes first sends zero base velocities and holds the arm where it is. No mode starts with motion.
- Auto Catch and Auto Release are one-shot actions that return to Manual Mode when they finish, fail their precondition, or are stopped.
- A press while an automatic action is running is ignored; `Stop` always works.
- The signboard shows the current mode in large text (Manual or FSC first; richer artwork later), the running action, and for a rejected Auto Catch or Auto Release the reason ("No egg in view", "Egg too far", "Egg too close", "Egg lost", "Could not align", "Basket not in view") for a few seconds in the warning color. There is never a dialog to dismiss: attendees have no cursor.

### Emergency stop behavior

`Stop` (or ESC / closing the signboard, or Ctrl+C) is a full stop and full reset (owner decision): it zeroes the base, cancels any automatic action, discards pending encoder rotation, clears voice input, and holds the arm at its current pose with torque kept on. After `Stop` the application is latched in a stopped state: the base stays at zero and the arm stays held, `Hi!` and `Thx` are ignored, and the signboard shows "STOPPED. Press Go Go! to resume" until `Go Go!` is pressed. That press returns to Manual Mode (it does not toggle to FSC) with arm following disengaged; following then resumes through the slow engagement below. Nothing moves again on its own after a stop (owner decision, 2026-09-24).

Arm torque is deliberately not released on `Stop`: the SO-ARM101 has no brakes, so a limp arm would fall under gravity, drop a held egg, and could hit the field or a hand. The ZMQ action protocol also carries only positions and velocities; the host releases torque only when it disconnects (`disable_torque_on_disconnect`). If staff need to move the arm by hand, they stop the host process instead. This stop travels through the signboard process and the control loop, so it is a software stop measured in tens of milliseconds; the Pi host's 500 ms command watchdog remains the last line.

## 3. Manual Mode details

- Base: exactly the Drive Mode behavior already implemented (joystick translation, encoder rotation, fixed speed level, zero velocities on input loss).
- Arm: follows the leader arm (`so100_leader`, id `dino_leader_arm`) joint for joint, including the gripper (the mouth). Leaving the leader arm resting at its home position keeps the dinosaur arm at home; this is the operating rule (owner decision), and there is no separate "go home" button.
- Engagement (owner decision): when Manual Mode starts, or after a stop, the follower moves slowly toward the leader's pose at `ARM_ENGAGE_SPEED_DEG_S` per joint. Real-time following begins only once every joint is within `ARM_ENGAGE_TOLERANCE_DEG` of the leader. The signboard shows "arm syncing" until then. This removes the jump a mismatched leader would otherwise cause.
- If the leader arm cannot be read (disconnected, bus error), the follower holds its last commanded pose and the signboard shows the fault; the base keeps working.
- The shaft button on the dino-controller has no role (`SHAFT_BUTTON_ROLE = "none"`): Catch moved to `Hi!`, stop is the second KachiButton. It stays wired and reported for a later use.

## 4. Auto Catch and Auto Release details

### Auto Catch

1. Precondition, evaluated on the front camera at the press: an egg is detected and its bounding-box height, as a fraction of the frame height, lies between `AUTO_CATCH_MIN_EGG_H` (too far) and `AUTO_CATCH_MAX_EGG_H` (too close). Otherwise "No egg in view", "Egg too far", or "Egg too close" and nothing moves. The best position is stored as the normalized bbox center and height (`ALIGN_TARGET_*`) measured from a reference frame captured on the robot with the egg placed where the owner wants it; the signboard draws that position as an egg-shaped outline at all times in Manual Mode so it also guides manual driving (owner suggestion, 2026-09-24).
2. Alignment (owner decision): the front camera gives the egg's position relative to the body. An image-based controller drives the base until the egg reaches the predefined target position and size in the front image, the one the pick policy was trained from. This step is what raises the catch success rate; the pick policy never has to compensate for base placement.
3. Catch: the pick policy runs on the wrist camera only (owner decision, consistent with the golf-ball result). Success is checked before the signboard says "caught".

### Auto Release

1. Precondition: the pink basket is detected in the front camera at or above `AUTO_RELEASE_MIN_BASKET_H` (normalized height). Otherwise "Basket not in view".
2. The arm moves to the home/base pose (neck folded) while keeping the gripper angle unchanged so the egg stays held.
3. A recorded release motion (a fixed joint trajectory, not a learned policy) delivers the egg into the basket and opens the mouth, then returns to home.

Egg colors (owner decision, 2026-09-24): white eggs with **green, red, or orange** spots. Blue spots were dropped that day because on the robot they measured the same hue, saturation, and value as the blue tarp; orange is added by painting. The detector is local (no network call), anchored on the spots found as edge ellipses surrounded by white egg body, classified by spot color, with the pink basket excluded by color. Thresholds are calibrated from captures on the robot and stored in the vision config.

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
| 1 | Mode manager, KachiButton phrase detection through the signboard (`Go Go!`, `Hi!`, `Thx`, `Stop`), Manual Mode with leader arm and slow engagement, mode text on the signboard. Auto Catch, Auto Release, and FSC show "not available yet" and do not move the robot | Owner drives and puppets, alone and with a second person; the four phrases change the display and `Stop` zeroes the base |
| 2 | Front-camera egg and basket detectors with venue calibration; Auto Catch precondition and base alignment; Auto Release precondition, home pose, and recorded release motion | Alignment ends with the egg at the target image position; release delivers a held egg into the basket |
| 3 | Policy runner for the trained `pick_egg` checkpoint (wrist camera), success check, one automatic retry | Auto Catch succeeds from a `Hi!` press |
| 4 | FSC: overhead camera, Gemini ER 2 target selection with rectangle confirmation, voice in and out, drive to the egg, return by pink beacon | End-to-end run from a spoken request to a released egg; duration measured |

## 7. Open points

- Home pose values and the release motion: recorded on the robot in Phase 2.
- Engagement speed and tolerance: tuned on the robot in Phase 1.
- Speech device, speech-to-text engine, and TTS for FSC progress (Phase 4).
