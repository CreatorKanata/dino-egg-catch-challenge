<!-- docs/project-status.md: Where the application stands, how to run, record, and train it, and what the road to Full Self-Catching looks like. -->
# Project Status and the Road to Full Self-Catching

Written: 2026-09-25. This is the working summary for the next development stage. Decisions live in
[spec/operating-modes.md](spec/operating-modes.md); the runtime facts and module layout live in
[lekiwi-app-development.md](lekiwi-app-development.md); the FSC design proposal is
[proposals/dino-egg-catch-challenge-gemini-integration.md](proposals/dino-egg-catch-challenge-gemini-integration.md).
Every statement about robot behavior below was owner-tested on 2026-09-24/25 unless marked *not yet run*.

## 1. What works today

| Area | State | Notes |
| --- | --- | --- |
| Manual Mode | Works on the robot | Base by dino-controller (joystick translation, encoder rotation 9° per click), arm by leader arm with slow engagement; one or two players |
| KachiButton controls | Works | `Go Go!` Manual/FSC toggle, `Hi!` Auto Catch, `Thx` Auto Release, `Stop` latch on a second unit; signboard window must keep focus |
| Signboard | Works | pygame in its own interpreter, 16:9 overhead + front/wrist, egg/basket outlines, notices; `c` capture, `b`/`k`/`r` staff keys |
| Egg detection | Works: green, red, yellow | Spot-anchored, edge fence + hull cap, ~13-17 ms/frame; weak spots: shaded lower half shortens the box, flicker when a same-colored object stands right behind the egg |
| Auto Catch alignment | Works | Base aligns on ellipse width (target 0.607) and center x; egg kept on the signboard guide |
| Auto Catch pick policy | Works, still rough | ACT (`CreatorKanata/act_dino_pick_egg`, 20 green episodes at ~8 fps wrist); caught the egg on the robot; temporal ensembling on; success is not verified ("Catch finished") |
| Auto Release | Works | Basket alignment (width), release pose, recorded 14.9 s motion, playback 1.0x |
| Camera controls on the Pi | Works | `start_dino_host.sh` fixes wrist 30 fps + focus 100 and front manual exposure; without it the wrist drops to ~8 fps |
| Network | Dedicated robot network | Pi at `192.168.2.9`; the phone hotspot caused shared frame stalls |
| Wrist-view check | Diagnostic only | Not a gate (owner decision): the angled wrist camera may not see a far-but-aligned egg |
| Success verification | *Not implemented* | "Caught!" is reserved until the egg-in-mouth check exists |

## 2. Daily commands

Pi (fork checkout, env `lerobot312`):

```bash
cd ~/lerobot-dino-egg-catch-challenge && git pull
./examples/lekiwi/start_dino_host.sh          # camera controls, then lekiwi_host --robot.id=dino_kiwi
```

Mac (this repository, env `lerobot312`, fork installed editable with `[lekiwi,viz,dataset]`; after any lerobot extra install
reinstall `opencv-contrib-python-headless==4.13.0.92`):

```bash
PYTHONPATH=src python -m robot.teleop_drive                      # Manual Mode + signboard, pick policy from the Hub
PYTHONPATH=src python -m robot.teleop_drive --rerun              # plus the Rerun operator view
# flags: --no-camera --no-signboard --fullscreen --controller-port --remote-ip --leader-port --no-leader
#        --pick-policy <hub id | dir | ''> --policy-device auto|mps|cpu --pick-ensemble 0.01 | --no-pick-ensemble
PYTHONPATH=src python -m unittest discover -s tests/robot         # 494 hardware-free tests
PYTHONPATH=src python -m robot.vision.inspect captures/<stamp>-front.png --debug   # detector diagnostics (--basket, --wrist, --rgb)
```

Signboard keys (staff): `c` capture all cameras to `captures/`, `b` save release pose, `k` save catch pose, `r` record/stop the release
motion, ESC end the session (base zeroed).

## 3. Recording and training

Recorder (app closed; Pi host running; leader arm connected; `hf auth login` done once):

```bash
PYTHONPATH=src python -m robot.recording.record_pick_egg --egg-color green  --num-episodes 20
PYTHONPATH=src python -m robot.recording.record_pick_egg --egg-color red    --num-episodes 20 --resume
PYTHONPATH=src python -m robot.recording.record_pick_egg --egg-color yellow --num-episodes 20 --resume
# dataset default CreatorKanata/dino_pick_egg_v2; --no-push --root /tmp/x for a trial; --voice none to mute
```

Episode rule (owner decision): from the catch pose, grasp, return to the catch pose, hold one second, right arrow. Left arrow discards
the episode, Esc ends the session (saves and pushes). Place the egg with deliberate offsets inside the alignment tolerance and mix
lighting. Failed episodes saved by mistake: `lerobot-edit-dataset --repo_id CreatorKanata/dino_pick_egg_v2 --operation.type delete_episodes
--operation.episode_indices "[i, j]"`, then push again.

Training (Google Colab Pro+, A100; open from GitHub after pushing the fork):
`https://colab.research.google.com/github/CreatorKanata/lerobot-dino-egg-catch-challenge/blob/main/examples/notebooks/dino_pick_egg_colab.ipynb`

- Secrets: `HF_TOKEN` (write), `SLACK_WEBHOOK_URL` (optional); switch both on for the notebook.
- Configuration cell: `DATASET_REPO_ID` (v2), `CAMERAS=["wrist"]`, `EPOCHS=12` (60 x 20 s episodes → ~54,000 steps, ~1 h on A100),
  `EVAL_SPLIT=0.1`, `IMAGE_AUGMENT=True`, `RUN_FINETUNE=False`.
- Run all; Slack reports start, finish (losses, push target), or failure. The model lands in `CreatorKanata/act_dino_pick_egg`;
  the app loads it at the next start. Resume cell only acts on an unfinished run.
- Venue plan: record ~10 episodes per color on site, set `RUN_FINETUNE=True` with `FINETUNE_EPISODES` = the venue indices,
  `FINETUNE_STEPS=15000`; output `CreatorKanata/act_dino_pick_egg_venue`; start the app with `--pick-policy CreatorKanata/act_dino_pick_egg_venue`.
- Step budget rule: think in epochs (ACT 8-16). 18,000 steps on 20 episodes was 12 epochs and took 20 minutes.

## 4. Data and model registry

| Item | Location |
| --- | --- |
| Demonstrations v1 (20 green, wrist ~8 fps, autofocus) | Hub dataset `CreatorKanata/dino_pick_egg` |
| Demonstrations v2 (green/red/yellow, 30 fps cameras) | Hub dataset `CreatorKanata/dino_pick_egg_v2` |
| Pick policy (v1 data) | Hub model `CreatorKanata/act_dino_pick_egg` (public) |
| Poses and motion | `data/arm/home_pose.json` (release pose, head up), `catch_pose.json`, `release_motion.json` |
| Camera settings | fork `examples/lekiwi/dino_camera_settings.sh`, udev rule `99-dino-cameras.rules` |
| Robot-side captures for detector tuning | `captures/` (git-ignored); the check list is in `src/robot/vision/config_vision.py` |

## 5. Tuning knobs worth knowing

`src/robot/config.py`: alignment (`ALIGN_TARGET_W_EGG` 0.607, tolerances, speeds), `RELEASE_PLAYBACK_SPEED` 1.0, arm engagement
(30 deg/s, 3°), KachiButton phrases. `src/robot/vision/config_vision.py`: HSV ranges per color, basket ranges, shape thresholds.
`src/robot/policy/config_policy.py`: `PICK_POLICY_PATH`, `PICK_TEMPORAL_ENSEMBLE_COEFF` 0.01, `PICK_MAX_S` 20, gripper-closed
threshold 12 %, per-joint step cap. `src/robot/recording/config_recording.py`: dataset id, gate tolerance, voice.

## 6. Road to Full Self-Catching (Phase 4)

FSC = spoken request → Gemini Robotics ER 2 picks the egg on the overhead image → rectangle shown, confirmed by voice (`Hi!` toggles
listening each time) → drive until the egg is in the front camera → Auto Catch → drive back by the pink basket beacon → Auto Release.
Everything after "egg in the front camera" already exists. What is missing, in the order to build it:

1. **Overhead scene tracker** (`vision/`): detect eggs and the robot on the 1280x720 overhead frame; robot pose from an AprilTag on the
   base; field calibration (tape points → homography); expose `target_id` tracking so a confirmed egg keeps its identity while the base
   moves. Acceptance: egg and robot positions stable to a few centimeters across the field, verified with captures.
2. **Gemini ER 2 adapter** (`gemini/`): non-streaming Interactions API first; input = overhead JPEG + request text; output = candidate
   boxes (`bbox_yxyx_1000`) validated by the app and matched to tracker IDs; no motion commands from the model. Cost and latency logged
   per call; a per-session budget stops new calls when exceeded.
3. **Speech**: push-to-talk by `Hi!` toggle (owner decision); speech-to-text engine to choose (local Whisper-class model vs Gemini Live);
   TTS for short status phrases; the signboard shows "Listening…", "Thinking…", "Is this the egg?" with the rectangle, "Confirmed".
4. **Navigation to the egg**: from the overhead pose, drive toward the target with the existing base controller until the front camera
   sees an egg whose identity matches the target (color plus predicted position); then hand over to Auto Catch alignment. Walls and other
   eggs are avoided with a simple planner on the calibrated field.
5. **Return by beacon**: rotate until the pink basket is in the front camera, drive until the basket width reaches the Auto Release
   target, run Auto Release; fallback to the overhead pose estimate when the basket is occluded.
6. **Mode manager**: FSC states (idle, listening, proposing, confirmed, driving, catching, returning, releasing) with `Stop` everywhere,
   `Thx` unused in FSC, one automatic action at a time, and the signboard phase texts.

Open decisions before step 3: speech-to-text engine, TTS voice, and whether FSC runs only as a staff demonstration (owner: yes, for now).
Prerequisites to check on the robot first: the front camera view with the head up and an egg held (must not be blocked), the overhead
camera's coverage of the whole field at 16:9, and the AprilTag size that the overhead camera resolves.

## 7. Before the venue

- Re-record with the 30 fps camera settings (60 episodes) and retrain; then the venue fine-tune.
- Egg-in-mouth success check (wrist or front camera after the catch) so "Caught!" can be announced honestly.
- Detector: shaded lower half of the egg, flicker with same-colored backgrounds; a lemon-yellow paint without text would help.
- `POWER_LINE=2` and shorter front exposure at the 60 Hz venue; re-check the wrist focus under venue light.
- Wired or dedicated 5 GHz link between the Mac and the Pi; the venue Wi-Fi is not to be used for the robot.

## 8. Venue arrival checklist (run this with the assistant on site)

When the owner says "we are at the venue", work through this list in order; each step names the capture or command that gives the
evidence. Budget about 60 minutes before opening. Steps 1-4 can run while the field is being set up.

1. **Network and host.** Pi and Mac on the dedicated link; `ping` the Pi (jitter under 10 ms). Start the host with
   `POWER_LINE=2 FRONT_EXPOSURE=<venue> ./examples/lekiwi/start_dino_host.sh`; start with `FRONT_EXPOSURE=150` and adjust so the front
   image is neither dark nor clipped (check a `c` capture). Wrist: keep `exposure_dynamic_framerate=0`; if the wrist image is dark, raise
   `WRIST_GAIN` before touching exposure.
2. **Frame rate sanity.** Watch the signboard for stutter; on the Pi run the `v4l2-ctl --stream-mmap --stream-count=150` check for both
   cameras (with the host stopped) and confirm ~30 fps. Record the values in this document's section 4 table afterwards.
3. **Overhead camera.** The whole field (walls to walls) is visible in the top view at 16:9; re-aim if not. Mark the robot parking spot and
   the egg best position with tape under the venue light.
4. **Detector captures, per color (green, red, yellow).** With the app running and the egg at the best position, press `c` for each of:
   best position, far (about 2x distance), with the pink basket directly behind, and, if the venue light varies (sun through a door,
   stage lights), once per lighting condition. Also one capture with no egg and one with the basket only. Then run
   `PYTHONPATH=src python -m robot.vision.inspect captures/<stamp>-front.png --debug` on each: exactly one egg of the right color, or none for
   the negatives. If a color fails, the assistant adjusts `src/robot/vision/config_vision.py` (HSV ranges, `EGG_MIN_SOLIDITY`, ring
   thresholds) against the full capture set, re-runs the unit suite, and commits with the venue stamps listed in the check comment.
5. **Basket.** `inspect --basket` on the basket-behind and basket-only captures: detected, `size ok` at the release position; adjust
   `BASKET_DETECT_HSV` if the venue light shifts the pink.
6. **Alignment targets.** Place an egg at the best position, capture, and read the ellipse width from `inspect`; if it differs from
   `ALIGN_TARGET_W_EGG` (0.607) by more than 0.03, the best position moved with the new tape marks, so either re-place the mark or update
   the target. Same for the basket width at the release position (`RELEASE_TARGET_W`).
7. **Wrist focus.** At the catch pose, capture and check the wrist frame is sharp on the egg; adjust `WRIST_FOCUS` on the Pi if needed.
8. **Acceptance run.** Manual Mode drive for one minute; `Hi!` three times per color from different offsets (expect alignment, catch,
   return to the release pose); `Thx` twice into the basket; `Stop` once during a catch (arm holds, base zero, `Go Go!` resumes). Note
   failures with the capture stamps and the `captures/*-align.csv` traces for the assistant.
9. **Fine-tune (optional).** If the schedule allows, record ~10 episodes per color on site with the recorder, then run the notebook with
   `RUN_FINETUNE=True` and start the app with `--pick-policy CreatorKanata/act_dino_pick_egg_venue`. Keep the pre-event model as the
   fallback (`--pick-policy CreatorKanata/act_dino_pick_egg`).
10. **Before opening.** Signboard fullscreen and focused, KachiButtons plugged in (`Stop` unit tested), Rerun off, phone hotspot off,
    the leader arm parked at the release pose, and a fresh `c` capture set saved as the venue reference.

What to tell the assistant on arrival: the lighting (natural, stage, mixed), the approximate size of the field, which colors are in
play, and how much time remains. The assistant then drives steps 4-7 from the captures and reports what changed.
