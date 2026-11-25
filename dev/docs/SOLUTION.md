# UniSC SAR — Project Reference

**Team:** UniSC (University of the Sunshine Coast)  
**Competition:** IEEE Systems, Man, and Cybernetics Society (SMCS) 2026 — Search and Rescue Phase 1  
**Last updated:** 2026-07-28 (solo — branch `joe-solo`; work plan: `docs/ARCHITECTURE.md`)  
**Solution code lives in:** `controllers/proposed_solution/` — see that folder's `readme.md` for the pipeline and next-steps checklist.

This document covers everything that readme does not: the full repo layout, competition context, simulator details, sensor reference, coordinate system, marking system, and session history.

---

## Full Repository Structure

```
UniSC-SAR-Competition/
│
├── controllers/
│   ├── proposed_solution/          ← OUR CODE (see readme.md inside)
│   │   ├── proposed_solution.py    ← Webots robot controller (main entry)
│   │   ├── prepare_mission.py      ← Offline flyover video analysis
│   │   ├── requirements.txt        ← Python deps
│   │   ├── readme.md               ← Solution architecture + next steps
│   │   └── sim_logs/               ← Generated outputs (CSV + PNG)
│   │
│   ├── rosbot_sensors_actuators_example/
│   │   └── rosbot_sensors_actuators_example.py   ← Full sensor demo class
│   │
│   ├── simple_rosbot_sar/
│   │   └── simple_rosbot_sar.py    ← Working random-walk + obstacle avoidance example
│   │
│   └── sar_marking_supervisor/
│       └── sar_marking_supervisor.py  ← Supervisor that logs & sends results to marking server
│
├── worlds/
│   ├── basic_example.wbt           ← No victims; good for sensor testing
│   ├── basic_example_sar.wbt       ← Has victims; good for basic navigation testing
│   ├── small_world.wbt             ← Competition world (smallest)
│   ├── medium_world.wbt            ← Competition world
│   └── large_world.wbt             ← Competition world (largest, up to 30×30 m)
│
├── recordings/
│   ├── small_world_flyover.mp4     ← Pre-recorded UAV flyover (1920×1080, 30 fps)
│   ├── small_world_flyover.csv     ← IMU data for the flyover
│   ├── medium_world_flyover.mp4
│   ├── medium_world_flyover.csv
│   ├── large_world_flyover.mp4
│   └── large_world_flyover.csv     ← cols: timestamp,acc_x,acc_y,acc_z,w_x,w_y,w_z,comp_x,comp_y,comp_z
│
├── protos/
│   ├── Rosbot.proto                ← Custom ROSbot definition used in all worlds
│   ├── Victim.proto                ← Human victim character definition
│   ├── OriginMarker.proto          ← 1×1 m calibration mat definition
│   ├── meshes/                     ← .obj / .stl mesh files (robot parts, human figures)
│   ├── skins/                      ← .fbx animation skins for human figures
│   └── textures/                   ← .png / .jpg textures (robot, humans, ArUco mat)
│       ├── OriginPattern.png       ← The ArUco + chessboard texture applied to the mat
│       └── OriginPattern.svg       ← Source SVG for the mat pattern
│
├── docs/assets/
│   ├── sar_example.png             ← Screenshot showing robot + victim layout
│   ├── example_map.png             ← Shows correct map_estimate.png format (left=world, right=expected output)
│   └── OriginPatternExplanation.svg
│
├── .venv/                          ← Python virtual environment (not committed)
├── README.md                       ← Official competition README (do not modify)
└── SOLUTION.md                     ← This file
```

> **Git LFS:** The `.mp4` recordings are stored in Git LFS. Run `git lfs install` before cloning, otherwise the video files will be pointer stubs.

---

## Competition Context

### What Phase 1 asks for
Teams submit a single Python-based Webots controller. A pair of ROSbot ground robots must autonomously find, navigate to, and report victims in a simulated disaster scene within **3 minutes**. Two components are assessed:

1. **Flyover analysis** (done offline before the sim): extract victim locations and a wall map from a pre-recorded drone video of the scene.
2. **Ground mission** (done in real-time): control both robots to find victims, navigating the scene using onboard sensors.

### What advances to Phase 2
Top 6 scoring teams from Phase 1 move to Phase 2, which uses **real physical robots**. All algorithms must therefore be robust enough to translate from simulation to hardware.

### Submission format
Submit a GitHub repo link to `cgonzalezarango "AT" swin "DOT" edu "DOT" au` with subject `2026 IEEE SMCS SAR Competition Submission`. The judges clone your repo into `controllers/proposed_solution/` and run it. Deadline: end of Phase 1 (date TBC).

---

## Simulator: Webots R2025a

Download from [cyberbotics.com](https://www.cyberbotics.com/#download).

### Opening a world
`File → Open World` → navigate to `worlds/` → open any `.wbt` file.

### Running a simulation
Press **▶ Play** (real-time) or **▶▶ Fast forward** (Ctrl+5) to speed up.  
Press **■ Reset** to restart without closing.

### Marking is automatic
When the simulation ends (all victims found OR 3-minute timer expires), the supervisor controller sends a log to the remote marking server and the score report is printed to the Webots console.

### Useful optional rendering
- `View → Optional Rendering → Show Lidar Point Cloud` — visualise lidar output in 3D
- `View → Optional Rendering → Show Camera Frustums` — see camera field of view

---

## Python Setup

Python 3.10+ is required. Install from [python.org](https://www.python.org/downloads/).

### macOS

```bash
python3 -m venv .venv --copies
.venv/bin/pip install -r controllers/proposed_solution/requirements.txt
```

**Webots Python command** (`Tools → Preferences → General → Python command`):
```
/Library/Frameworks/Python.framework/Versions/3.XX/bin/python3.XX
```
Replace `3.XX` with your installed Python version (e.g. `3.14`).

**Why not the venv path?**  
The venv's `.venv/bin/python3` binary gets a `com.apple.provenance` extended attribute that Webots's subprocess launcher rejects — even when the file is a real binary created with `--copies`. The Python.org framework binary at the path above is clean. The `proposed_solution.py` controller adds `.venv/lib/*/site-packages` to `sys.path` at startup so packages are found regardless.

*Verified on: macOS (Apple Silicon), Python 3.14, Webots R2025a — 2026-06-21.*

### Windows

```cmd
python -m venv .venv
.venv\Scripts\pip install -r controllers\proposed_solution\requirements.txt
```

**Webots Python command**:
```
C:\path\to\UniSC-SAR-Competition\.venv\Scripts\python.exe
```
Use the full absolute path. Windows venvs use real executables (no symlinks), so the venv path works directly with Webots — no framework binary workaround needed.

### Both platforms — venv structure difference

The `sys.path` preamble in `proposed_solution.py` handles both:

| Platform | Venv site-packages location |
|---|---|
| Windows | `.venv\Lib\site-packages\` |
| macOS / Linux | `.venv/lib/pythonX.Y/site-packages/` |

### Git LFS (required on all platforms)

The `.mp4` flyover recordings are tracked by Git LFS. Without it, cloning gives empty pointer stub files instead of real videos.

```bash
git lfs install   # run once after installing git-lfs
git pull          # re-fetch LFS files if already cloned without it
```

Install Git LFS: [git-lfs.com](https://git-lfs.com) · macOS: `brew install git-lfs` · Windows: download installer from site.

---

## Coordinate System

The world uses a right-handed coordinate frame anchored to the **origin marker mat**:

```
         +Y (north on mat)
          │
          │
          └──────── +X (east on mat)
         /
        /
       +Z (up, out of ground)
```

| Reference point | Position |
|---|---|
| World origin | Centre of ArUco Marker 0 on the mat |
| Robot 1 start | (−0.375, +0.375) m |
| Robot 2 start | (−0.375, 0.000) m |
| Mat extent | Roughly −0.5 m to +0.5 m in X and Y |
| Max world size | 30 m × 30 m |

### Origin marker mat
- Size: **1 × 1 m** white mat
- Has three **4×4 ArUco markers** (dictionary: `DICT_4X4_50`, physical size: 0.25 × 0.25 m each):
  - **Marker 0** — centre = world origin (0, 0)
  - **Marker 2** — in the +X direction from Marker 0
  - **Marker 1** — in the +Y direction from Marker 0
- Also has a **9×6 chessboard** (tile size 45 mm) for camera intrinsic calibration

---

## Flyover Recordings

Each recording starts with a **vertical takeoff from above the origin mat**, then follows a predefined horizontal route at varying altitude (altitude drops when inside low-ceiling areas).

### Video format
- Resolution: 1920 × 1080 px
- Frame rate: 30 fps
- Camera: downward-facing (always pointing at the ground)

### IMU CSV format
```
timestamp, acc_x, acc_y, acc_z, w_x, w_y, w_z, comp_x, comp_y, comp_z
```
- `acc_*` — accelerometer (m/s², includes gravity ≈ 9.81 in Z when hovering)
- `w_*` — gyroscope (rad/s)
- `comp_*` — compass unit vector (pointing to magnetic north in sensor frame)

Sensor model: simulated MPU-9250. Timestamps are in seconds.

---

## Sensor & Actuator Reference

All device names are strings passed to `robot.getDevice("name")`.

### Motors
| Name | Wheel |
|---|---|
| `fl_wheel_joint` | Front left |
| `fr_wheel_joint` | Front right |
| `rl_wheel_joint` | Rear left |
| `rr_wheel_joint` | Rear right |

Set to velocity mode: `motor.setPosition(float("inf"))` then `motor.setVelocity(speed_rad_s)`.

### Wheel position sensors (encoders)
| Name | Wheel |
|---|---|
| `front left wheel motor sensor` | Front left |
| `front right wheel motor sensor` | Front right |
| `rear left wheel motor sensor` | Rear left |
| `rear right wheel motor sensor` | Rear right |

Enable with `sensor.enable(timestep)`. Read with `sensor.getValue()` → radians.

### RGB + Depth Camera (Orbbec Astra, forward-facing)
| Device name | Type | Notes |
|---|---|---|
| `camera rgb` | Webots Camera | `getImage()` returns BGRA bytes |
| `camera depth` | Webots RangeFinder | `getRangeImage()` returns float list |

Convert RGB image:
```python
buf = camera_rgb.getImage()
img = np.frombuffer(buf, np.uint8).reshape((h, w, 4))
rgb = img[:, :, :3][:, :, ::-1]   # BGRA → RGB
```

### Lidar (Slamtec RpLidarA2)
| Device name | Notes |
|---|---|
| `laser` | 360° scan, returns range image (list of floats in metres) |

Enable: `lidar.enable(timestep)` + optionally `lidar.enablePointCloud()`.  
Read: `lidar.getRangeImage()` — list of distances, first element is forward.

### IMU (TDK MPU-9250)
| Device name | Method |
|---|---|
| `imu accelerometer` | `.getValues()` → [x, y, z] m/s² |
| `imu gyro` | `.getValues()` → [x, y, z] rad/s |
| `imu compass` | `.getValues()` → [x, y, z] unit vector |

Heading from compass:
```python
cx, _, cz = compass.getValues()
heading = math.atan2(cx, cz)   # radians
```

### Short-range IR distance sensors (max 2 m)
| Name | Position |
|---|---|
| `fl_range` | Front left |
| `fr_range` | Front right |
| `rl_range` | Rear left |
| `rr_range` | Rear right |

Read: `sensor.getValue()` → metres.

### Communication devices
| Device name | Channel | Purpose |
|---|---|---|
| `supervisor emitter` | **43 only** | Send victim-found notifications to marking supervisor |
| `robot to robot emitter` | 73 (default) | Send messages to other robots |
| `robot to robot receiver` | 73 (default) | Receive messages from other robots |

> **Do not use channel 43 for anything other than victim-found messages.** The marking supervisor listens exclusively on that channel.

---

## Victim-Found Notification

Must be sent via `supervisor emitter` (channel 43). Incorrect format = no score for that victim.

```python
import json

message = {
    "timestamp":         robot.getTime(),     # float, simulation seconds
    "robot_id":          robot.getName(),      # "robot1" or "robot2"
    "position":          [x, y, 0.0],          # your odometry estimate (metres)
    "victim_found":      True,
    "victim_confidence": 0.95                  # float [0, 1]
}
supervisor_emitter.send(json.dumps(message).encode())
```

**Rules:**
- Robot must be **within 1.0 m** of the victim's centroid (projected to ground) to score.
- High `victim_confidence` that matches actual outcomes boosts the confidence score.
- Each victim scores only once — duplicate reports are ignored.

---

## Marking System

### How it works
1. The `sar_marking_supervisor` controller (separate process in the world) logs all robot actions and victim-found messages throughout the simulation.
2. When the simulation ends, it sends the log as JSON to the remote marking server: `https://ieee-sar-hackathon.ts.r.appspot.com/mark-log`
3. The server returns a score report (printed to the Webots console).

### Marking report structure
```
OVERALL SCORE: X.XXX/1.000

COMPONENT SCORES:
  Victim Finding       (40%)
  Efficiency           (15%)
  Coordination         (15%)
  Video Info Extraction(30%)
    └── victim_location_estimates.csv  (half of 30%)
    └── map_estimate.png               (half of 30%)

INPUT VALIDATION WARNINGS  ← read this section carefully
RECOMMENDATIONS
TOP PRIORITY AREAS
```

### Sending logs manually (for debugging)
You can post a log file to the marking server without running a simulation:
```python
import json
from urllib import request

with open("log/<log-file>") as f:
    log = json.load(f)

req = request.Request(
    "https://ieee-sar-hackathon.ts.r.appspot.com/mark-log",
    data=json.dumps(log).encode(),
    headers={"Content-Type": "application/json"},
    method="POST"
)
with request.urlopen(req) as resp:
    print(resp.read().decode())
```

---

## Example Controllers (for reference)

### `rosbot_sensors_actuators_example.py`
Run with `worlds/basic_example.wbt`. The `Rosbot` class wraps every sensor and actuator with clean helper methods. Use this as a reference when adding new sensor reads to `proposed_solution.py`.

### `simple_rosbot_sar.py`
Run with `worlds/basic_example_sar.wbt`. Implements random-walk exploration with distance-sensor obstacle avoidance. The `BasicRosbotController` class shows the minimal boilerplate needed for a working controller (motor init, sensor enable, step loop).

---

## Session History

### 2026-06-21 (Session 1)
**What was accomplished:**
- Explored the full repository and understood competition requirements.
- Decided on a two-phase strategy: offline video analysis feeding into online waypoint navigation.
- Installed `opencv-contrib-python 4.13` into the project venv.
- Built `prepare_mission.py` from scratch (ArUco calibration, ORB tracking, HSV victim detection, greedy clustering, Hough-line wall map).
- Built `proposed_solution.py` (preflight call, odometry, proportional navigation, victim reporting).
- Created `requirements.txt` and `readme.md` (solution architecture + next-steps checklist).
- Ran into macOS Webots Python path issue — `.venv/bin/python3` has `com.apple.provenance` attribute that blocks Webots from executing it.
- Recreated venv with `--copies` flag and Python 3.14; added `sys.path` preamble to controller.
- Issue not fully resolved by end of session. Root cause: Webots needs the framework binary path directly, not the venv path.
- Installed `git-lfs` via Homebrew (was missing, caused `git status` to fail).
- Committed all new files. Push deferred to user (requires GitHub credentials).

**What was NOT done:**
- `prepare_mission.py` has not been run on actual video footage — outputs are untested.
- Webots simulation was never successfully started — Python path issue blocked it.
- Robot navigation has not been tested in simulation.
- No tuning of detection thresholds has occurred.

---

### 2026-06-22 (Session 2)
**Platform:** Windows 11, Python 3.14.5, Webots R2025a.

**What was accomplished:**

**Environment fixed (Windows):**
- Created `.venv` at project root (`python -m venv .venv`) and installed `numpy 2.5.0`, `opencv-contrib-python 4.13.0`, `pillow 12.2.0`. The existing `sys.path` preamble in `proposed_solution.py` then found packages correctly.
- Added the same `.venv` site-packages injection preamble to `sar_marking_supervisor.py` — it was missing entirely, causing `ModuleNotFoundError: No module named 'numpy'` on that controller.

**`prepare_mission.py` — singular matrix crash fixed:**
- ORB tracking drift caused `cum_H` to accumulate into a degenerate (non-invertible) matrix over many frames. `np.linalg.inv(cum_H)` then threw `LinAlgError: Singular matrix`, crashing the whole preflight.
- Fix: added `|det| < 1e-10` guard on every `H_inter` update (bad updates are rejected) and a per-frame skip when `cum_H` itself is still singular.
- Cleaned up unused `prev_gray` variable from the tracking loop.

**`proposed_solution.py` — obstacle avoidance overhauled:**
- Added `_min_front_range()`: returns minimum finite range in the front 120° lidar sector, filtering out `inf`/max-range sentinel values.
- Added `_clear_side()`: compares left vs right lidar sectors and returns whichever has more clearance.
- Both navigation and wander modes now **back up first** when < 0.25 m from a wall, then steer toward the clearer side — previously the robot always turned the same direction and would grind into walls or loop back into them.
- Raised obstacle detection threshold from 0.4 m → 0.6 m (more reaction distance).
- Increased forward speed: wander 60% → 90% of MAX_SPEED; navigation 70% → 80%.

**First successful simulation run on small_world.wbt:**
- Both robots initialised, preflight video analysis ran without crashing, 5 pre-computed waypoints loaded, robots moved through the scene with working avoidance.

**Issues discovered in Session 2:**
- The 5 pre-computed waypoints are **false positives near the ArUco mat** (radius ~0.7–1.0 m from origin). Actual victims are 2–10 m away in the ArUco frame. The `ORIGIN_EXCL_M = 0.7` exclusion radius is not wide enough — the mat's bright-coloured regions or the robots themselves leak through the colour segmentation.
- `[SUPERVISOR] MSG RECEIVED — Found verdict: False` for all victim reports — robots are not within 1 m of any real victim when they report.
- Robots exhaust their (false) waypoints almost immediately and spend the full 3 minutes in wander mode.

**What was NOT done:**
- Root cause of false-positive victim detections not fixed — `ORIGIN_EXCL_M` not yet widened and video frames not inspected visually.
- Camera-based on-robot victim confirmation not implemented.
- Waypoint routing not optimised.

---

### 2026-06-22 (Session 2 — continued, overnight code sprint)
**Platform:** Windows 11, Python 3.14.5, Webots R2025a.

**What was accomplished (code written, not yet tested):**

**`prepare_mission.py` — calibration accuracy improved:**
- `build_calibration_H()` rewritten to use all 4 corners of every detected marker as extra correspondences, not just the 3 marker centres.
  - A preliminary H is built from centres, then each image corner is projected through it, snapped to the nearest of the 4 known world corner positions (±0.125 m from marker centre), and added to the correspondence set.
  - With 3 markers: up to 15 point pairs for `findHomography` instead of 3. Much more over-determined → more accurate and robust calibration.
- ArUco re-anchor frequency changed from every 5 sampled frames to **every sampled frame** (every 1 second of video). The detection call is cheap and only fires when markers are actually visible, so overhead is negligible. This maximises the drift-correction window.

**`proposed_solution.py` — rewritten with 6 improvements:**
1. **Nearest-neighbour waypoint routing** (`_plan_route()`): replaces the even/odd index split. All waypoints are ranked by distance from each robot's start position; Robot 1 gets even-ranked waypoints, Robot 2 gets odd-ranked. Within each set, a nearest-neighbour greedy algorithm orders the visits. Improves Efficiency (15%) and Coordination (15%).
2. **Depth camera initialised** (`camera depth` device): enables the depth RangeFinder. `_camera_victim_confidence()` now projects the colour-segmented mask onto the depth image and rejects detections where the nearest valid depth exceeds 4 m (false positives from distant coloured objects). When within range, confidence is scaled up by proximity (victim at 1 m → ~50% confidence boost vs same colour blob at 4 m).
3. **Distance-based victim deduplication**: replaced `set` of rounded position keys with `self._reported_positions: list[(x,y)]` and `_already_reported_nearby(x, y, radius=2.0)`. Prevents double-reporting victims that are at the same real location but different rounded keys due to odometry drift.
4. **Odometry compass fallback fixed**: `update_odometry()` previously discarded the encoder-computed heading change `d_th` even when the compass was unavailable (the `self.theta = self.heading()` line returned stale `self.theta` when no compass). Now: `self.theta = self.heading()` when compass is present; `self.theta += d_th` otherwise.
5. **Robot-to-robot victim sharing**: when a robot confirms a victim, it broadcasts `{"type":"victim","x":...,"y":...}` via the `robot to robot emitter`. The partner reads this in `_read_squad_messages()` (called every timestep) and adds the position to its own `_reported_positions` so it won't double-report the same victim.
6. **`START_POS` constant dict** added — starting positions for each robot now defined once and reused by both `__init__` and `_plan_route()`, removing the duplicated inline values.

**What was NOT done / still needs testing:**
- None of the above improvements have been run in simulation — this was a code-only sprint.
- Video footage has not been visually inspected to understand what the HSV detection is actually picking up near the origin.
- HSV thresholds have not been tuned to the actual Webots victim appearances.
- Waypoint positions are still likely off due to ORB Y-axis drift — fixing this requires either better calibration (now improved with corners) or visual inspection + tuning.

---

### 2026-07-28 (Session 3 — solo consolidation, branch `joe-solo`)
**Project became solo (Joe).** All prior team work consolidated on `joe-solo` (off
`Bertus-Work`). Docs rewritten: `PLAN.md` is the work plan (phases/gates/
contingency), `docs/ARCHITECTURE.md` the architecture reference; the three
workstream briefs (JOE/BERTUS/WEBSTER.md) are deleted.

**Extraction (offline-validated with `tools/eval_harness.py`):**
- IMU radial-scale correction fixed medium_world's altitude-compression (victims 15.4→9.8 m,
  map IoU 0.010→0.043; small/large untouched by construction).
- Scale-gated wall-segment pruning (medium IoU →0.055). Negative results logged: full
  similarity/translation IMU fits and per-sample drift blending all regress — IMU supplies
  exactly one reliable DOF (global scale). See `docs/logged_attempts.md`.
- New internal output `nav_grid.npy` — origin-anchored wall grid for route planning.

**Controller (validated across five live small_world runs):**
- Fixed compass heading axis (Z-up ENU, `atan2(cx, cy)`) — robots had navigated in a frame
  rotated ~90°. Fixed WHEEL_RADIUS 0.05→0.043 (odometry now ~cm-accurate). Raised cruise
  speed 5→14 rad/s (~0.48 m/s) with speed-scaled steering.
- Added victim HOMING: supervisor scores only when the robot is truly <1.0 m, so the robot
  now drives to <0.8 m (denim-region depth) before reporting once at 0.9. Denim gate killed
  furniture false positives; failed homing keeps the waypoint (8 s cooldown).
- Added global A* route planning on nav_grid (erode 0.10 m/inflate 0.10 m) with lidar
  obstacle memory painting unseen walls into the grid and re-planning.
- Removed robot2's 8 s staggered start.

**Honest status:** robots navigate and report cleanly, but **0 victims scored yet** in live
runs — the detect→home→<1 m→report chain has not closed end-to-end. That is priority #1
(see ARCHITECTURE.md §8). Deadline: Friday 31/07/2026.

---

## For an AI Assistant Reading This

If you are an LLM picking up this project in a future session, read `docs/ARCHITECTURE.md` first (single source of truth), then this file for repo layout/sensor reference, then `controllers/proposed_solution/readme.md` for the code architecture.

**Critical facts to know:**
- This is a Webots R2025a simulation. The robot controller language is Python 3.10+.
- Both ground robots run `controllers/proposed_solution/proposed_solution.py`. They identify themselves via `robot.getName()` → `"robot1"` or `"robot2"`.
- All competition code must be in `controllers/proposed_solution/`. The judges clone that folder.
- No external API calls are allowed — everything must run offline.
- The Webots Python path on macOS must be set to the **framework binary**, not the venv: `/Library/Frameworks/Python.framework/Versions/3.14/bin/python3.14`. The controller adds the venv's site-packages to `sys.path` automatically.
- `git lfs install` must be run before `git pull/clone` or the `.mp4` files will be empty pointer stubs.
- The marking score report is printed to the Webots console at the end of each simulation run — look for `OVERALL SCORE` in the console output to assess performance.
- The highest-value thing to improve is **Victim Finding (40%)** — the robots reaching and reporting victims matters more than map accuracy.