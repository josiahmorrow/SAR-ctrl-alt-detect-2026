# 2026 IEEE SMCS SAR Competition — Phase 1
## Architecture & Work Plan (solo — Joe)

**Single source of truth. Read before writing any code.**
When any other doc disagrees with this file, this file wins. When this file disagrees
with the CODE (especially `sar_marking_supervisor.py`), the code wins — then fix this file.

- **Goal:** working, format-correct, tested submission by **Friday 31/07/2026**.
- **Solo project** on branch **`Google-Maps-inspired`** (off `joe-solo`, the untouched
  restore point). **The work plan (phases, gates, contingency) is `PLAN.md` (repo root).**
- Judged on **unseen worlds** — anything tuned to small_world specifically is worthless.

---

## 1. The mission in one paragraph
A drone has flown over each scene, recording downward video + IMU. Offline we estimate
victim positions and a wall map from that footage (30% of marks). Live, two ROSbots must
physically get within **1.0 m of each victim's marker** and send a channel-43 JSON while
there (40%), efficiently (15%), splitting the work evenly (15%). 3-minute limit.

## 2. Hard constraints (violating any costs the round)
- **Python 3.10+ only.** No compiled objects, no external API/server calls. Self-contained.
- Worlds ≤ 30×30 m. Exactly 2 robots. Mission ends at 3 min or all victims found.
- Entry point fixed: `controllers/proposed_solution/proposed_solution.py`.
- Victim-found on **channel 43 only** via `supervisor emitter` (§5c). Robot-to-robot on 73.
- **Scoring truth (from the supervisor code):** a report scores only if the robot's TRUE
  position is within 1.0 m of the victim marker when the message arrives, AND that victim
  isn't already found. The reported `position` field is logged, not scored. False reports
  poison the confidence sub-score.

## 3. Where the marks are
| Component | Weight | Notes |
|---|---|---|
| Victim Finding | 40% | ratio found + speed + confidence calibration |
| Video Information Extraction | 30% | mean of victim-location accuracy and wall-map accuracy |
| Efficiency | 15% | distance travelled + duration |
| Coordination | 15% | rewards an even victim split between the 2 robots |

**Traps (unchanged, still real):** over-reporting victims is penalised; confidence policy
is GATE-then-report-HIGH (0.9, once per victim, only when physically in range); the wall
map is where teams bleed.

## 4. Reference frame (everything shares this)
Metres, relative to the origin-marker mat (1×1 m). Origin = centre of ArUco Marker 0;
+X → Marker 2; +Y → Marker 1; +Z up. World is **Z-up ENU** — compass heading is
`atan2(comp_x, comp_y)` (the flyover IMU and the robot controller use the same convention).
Robot 1 starts (-0.375, 0.375); Robot 2 starts (-0.375, 0.0) — as children of the
OriginMarker node, so their world pose is mat pose + offset. No GPS anywhere.

## 5. Output contracts (EXACT — automated marking)
### 5a. victim_location_estimates.csv
- `controllers/proposed_solution/sim_logs/victim_location_estimates.csv`
- No header. One victim per line: `x,y` metres, origin frame, numeric only.
- One estimate per real victim — no duplicates, no speculative extras.
### 5b. map_estimate.png
- `controllers/proposed_solution/sim_logs/map_estimate.png`
- Exactly 600×600 px, 0.05 m/px. Walls pure black [0,0,0]; free pure white [255,255,255].
- Walls only (no furniture), 0.2 m (4 px) thick.
- **Window centred on the wall bounding box** — this matches the marking supervisor's
  ground-truth convention. (An earlier version of this doc said "origin at pixel
  300,300"; that was wrong — the supervisor code wins.)
### 5c. Victim-found notification (live, channel 43)
```json
{ "timestamp": 0.0, "robot_id": "robot1", "position": [1.2, -4.8, 0.0],
  "victim_found": true, "victim_confidence": 0.9 }
```
`supervisor_emitter.send(json.dumps(msg).encode())` — send once per victim, only when
physically within 1.0 m.

## 6. System shape (current, as built)
Two halves, one contract, all inside `controllers/proposed_solution/`:

```
flyover video + IMU
   └─ prepare_mission.py  (preflight, runs once inside proposed_solution.py)
        ├─ ArUco calibration → visual odometry → loop closure → mosaic refine
        ├─ IMU radial-scale correction (fixes altitude-compression; no-op when s≤1)
        ├─ victim detection (denim-first) → victim_location_estimates.csv   [graded]
        ├─ wall segments (+prune when scale-inflated) → map_estimate.png    [graded]
        └─ nav_grid.npy — origin-anchored 1200×1200 wall grid @0.05 m/px    [internal]

proposed_solution.py  (both robots)
   ├─ loads victim CSV → waypoints (distance-ranked, interleaved robot1/robot2)
   ├─ GLOBAL: A* routes on nav_grid (eroded 0.10 m to reopen over-drawn doorways,
   │   inflated 0.10 m) followed as ~0.6 m sub-waypoints; re-plan ≥ every 5 s
   ├─ LOCAL: lidar reactive layer — forward-biased drive, stuck-escape (reverse+turn),
   │   and OBSTACLE MEMORY: lidar-hit cells are painted into nav_grid → re-plan learns
   ├─ VICTIM: camera homing gated on BLUE DENIM (all victim models wear denim) with
   │   depth measured on the denim region; drive to <0.8 m, then report once @0.9;
   │   failed homing keeps the waypoint + 8 s cooldown
   └─ COMMS: ch-43 report; ch-73 victim broadcast + position; dedup within 0.5 m
```

Design principle: the flyover map is good enough to route THROUGH (which doorway), never
to steer by. Lidar is the only steering-grade sensor; the nav grid self-corrects as
robots hit walls the flyover missed.

## 7. Current measured status (28/07/2026 — be honest, update after every run)
**Extraction (offline, eval_harness):**
| World | Victim est. vs GT | Map IoU | Notes |
|---|---|---|---|
| small | 0.76/0.90/1.05/3.08 m (4/4) | 0.057 | victim4 pose-displaced |
| medium | mean 9.8 m (4/4) | 0.043 | residual odometry drift; scale fix applied ×2.81 |
| large | mean 2.67 m (4/5, 1 missed) | 0.024 | victim3 undetected at true position |

**Mission (small_world, live):** robots navigate, victims **0/4 found** so far. Fixed in
sequence: compass axis (was 90° off), speed (0.25→0.48 m/s), wheel radius (odometry now
~cm-accurate), false reports (denim gate), route planning (A*), obstacle memory. Next
verification run pending.

**Known residual issues:** waypoint timeouts when routes cross walls the map missed
(obstacle memory should now fix); victim4 (small) waypoint 3 m off — the sweep/scan must
cover it; medium victim waypoints ~10 m off (extraction drift — mission there depends on
sweep + camera, not waypoints).

## 8. Priority order to Friday — RUTHLESS
1. **First scored victim end-to-end on small_world** (the entire 40% hinges on the
   detect→home→<1 m→report chain working once; everything else is iteration).
2. **3–4 victims on small_world** reliably; even split (15%).
3. **Verify medium/large don't crash**; robots must still sweep+find without good
   waypoints. Judged worlds are unseen — sweep robustness is the floor.
4. **Submission hygiene:** clean clone install, preflight runs, formats verified,
   readme accurate, no external calls, email subject exact.
5. Only if time remains: extraction accuracy (victim4 outlier, map IoU), efficiency.

## 9. Environment & submission
- Webots R2025a, Python 3.10+ (dev on 3.14), Git LFS for recordings.
- venv: `numpy==2.4.6`, `opencv-contrib-python==4.13.0.92` (ArUco), `pillow==12.2.0` —
  canonical pinned file: `controllers/proposed_solution/requirements.txt` (root file
  redirects to it).
- macOS Webots Python command: the framework binary (not the venv path); the controller
  adds the venv site-packages itself. See `controllers/proposed_solution/readme.md`.
- Submission: repo link emailed to `cgonzalezarango AT swin DOT edu DOT au`,
  subject exactly `2026 IEEE SMCS SAR Competition Submission`.

## 10. Dev tooling (not part of the submission runtime)
- `tools/eval_harness.py` — offline scorer (GT from .wbt, replicates supervisor).
- `tools/tune_wall_prune.py`, `tools/tune_victim_merge.py` — offline sweeps.
- Env-guarded debug dumps in prepare_mission: `SAR_DUMP_SPREAD`, `SAR_DUMP_DETS`,
  `SAR_DUMP_POSES` (all inert unless the env var is set).
- `docs/logged_attempts.md` — failure log; check before re-trying an idea.
