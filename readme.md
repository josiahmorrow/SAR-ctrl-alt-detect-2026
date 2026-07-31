# ctrl+alt+detect — 2026 IEEE SMCS SAR Competition, Phase 1 Submission

University of the Sunshine Coast. Everything runs from `proposed_solution.py`;
flyover analysis is triggered automatically before the control loop starts, so
no separate preprocessing command is required.

Two extraction pipelines ship, each doing what it measured best at:
`flyover_extraction.py` writes the two GRADED deliverables (victim estimates and
wall map), and `prepare_mission.py` runs alongside it purely to produce the
controller's internal planning grid, writing its own estimates to
`sim_logs/uni_extract/` where they cannot be mistaken for the answer.

## Environment replication

Python **3.10 or newer** (developed and tested on 3.10–3.14). Webots R2025a.

```bash
python -m venv .venv
# Windows:  .venv\Scripts\activate      macOS/Linux:  source .venv/bin/activate
pip install -r requirements.txt
```

Point Webots at the venv python (`Tools > Preferences > General > Python
command`), open a world, press Play. Note: `opencv-contrib-python` (pinned in
`requirements.txt`) is required — the contrib build carries the ArUco module
used for camera calibration.

On first run for a world, the controller analyses the matching flyover video
(~2 minutes wall-clock; simulation time does not advance while controllers
initialise). Robot1 performs the analysis; robot2 waits for its outputs.
Results are cached in `sim_logs/` and reused until the recording or the
extraction code changes. Manual trigger, if preferred:

```bash
python flyover_extraction.py --video ../../recordings/<world>_flyover.mp4
python prepare_mission.py    --video ../../recordings/<world>_flyover.mp4
```

The IMU CSV beside the video is auto-detected. Outputs written to `sim_logs/`:
`victim_location_estimates.csv` (headerless `x,y` per line) and
`map_estimate.png` (600×600 @ 0.05 m/px, walls black, free space white), both
from `flyover_extraction.py`; plus `nav_grid.npy` (internal planning grid) from
`prepare_mission.py`, whose own estimates land in `sim_logs/uni_extract/`.

## Solution overview

**Victim and map extraction** (`flyover_extraction.py`, with `route_planning.py`):
victim positions are projected from the flyover IMU track with scale parameters
selected by recording length, then clustered and filtered to a submitted set.
This pipeline localises victims to a mean 0.30–0.61 m of the scored markers on
the development worlds, against 1.45–5.06 m for the alternative below — decisive
given the 1.0 m scoring radius.

**Planning-grid extraction** (`prepare_mission.py`): camera intrinsics are
calibrated from ArUco views of the origin mat. Pass 1 runs compass-constrained
visual odometry at 4 Hz with multi-pass loop closure and a global pose solve,
followed by mosaic re-registration. Because visual odometry pins metric scale
only at the mat, off-mat poses compress toward the origin; each posed sample
is corrected by its own radial gain fitted against the metric IMU trajectory
(a single global scale cannot fix the warp — the measured gain varies ×2–×7
across one flight). Pass 2 detects victims with a denim-colour detector
(primary, high precision) and a broad colour-blob detector (fallback
coverage), then clusters: denim clusters need ≥2 corroborating frames, extra
slots beyond four must be genuinely new areas, and broad clusters fill
uncovered areas subject to a vote floor. Walls are accumulated as a
per-cell-minimum brightness mosaic and extracted as axis-aligned segments; the
output map window is centred on the wall bounding box.

**Mission execution** (`proposed_solution.py`): both robots run the same
controller. The victim estimates are split between the robots by brute-force
partition minimising total A* route cost (deterministic on both robots, no
communication needed). Navigation is lidar-first: the routing grid starts
empty, a 3.5 s startup spin paints the surrounding walls, and lidar
continuously paints/erases the grid (soft-cost A*, walls crossable at high
cost — never eroded). Victim confirmation is a gated camera chain: denim blob
fraction, colour ratio, median depth, height-above-floor and physical-area
gates over a 4-frame streak, followed by a close-approach so the robot is
within 1.0 m before the channel-43 report (JSON: timestamp, robot_id,
position, victim_found, victim_confidence). Estimated-victim waypoints that
time out are requeued once; a robot that exhausts its own list adopts the
partner's unconfirmed estimates (channel-73 coordination) before falling back
to a building-bounded sweep for unestimated victims. The submitted map is
upgraded during the mission: both robots persist their lidar-learned grids,
which are merged and written over the flyover map late in the mission and
immediately before every victim report (the simulation ends the moment the
last victim is reported).

## Repository contents

| File | Purpose |
| --- | --- |
| `proposed_solution.py` | Ground-robot controller (both robots) + automatic preflight |
| `flyover_extraction.py` | Flyover video → **graded** victim CSV + wall map |
| `route_planning.py` | Route/waypoint helpers imported by `flyover_extraction.py` |
| `prepare_mission.py` | Flyover video → `nav_grid.npy` planning grid (its CSV/map go to `sim_logs/uni_extract/`) |
| `requirements.txt` | Pinned dependencies |
| `sim_logs/` | Extraction outputs (generated at runtime) |

## Team

ctrl+alt+detect, University of the Sunshine Coast — Josiah Morrow, Ezekiel Pari,
Alex Bertus, Alexandra Webster. Academic advisor: Dr Erica Mealy.
`flyover_extraction.py` and `route_planning.py` are Ezekiel Pari's work; the
controller, planning-grid extraction and integration are Josiah Morrow's.
