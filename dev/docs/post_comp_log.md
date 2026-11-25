# Post-competition log

Platform for every run below unless stated: Minisforum UM773 Lite, Windows 11
Pro, Python 3.14.5, numpy 2.4.6, opencv-contrib 4.13.0.92, Webots R2025a,
judges' layout (organisers' repo at `3fe3ca8`, supervisor and worlds
untouched), 180 s mission, hermetic runs via `dev/tools/run_batch.py`.

## Stage 1 - baseline (25/11/2026)

Controller code: `phase1-submitted` (`c6951a7`), unchanged.

| World | Runs | Victims found | False reports | Overall | Finding | Effic. | Coord. | Extraction | Victim est. err |
|---|---|---|---|---|---|---|---|---|---|
| small | 1 | 0/4 | 0 | 0.193 | 0.000 | 0.00 | 0.00 | 0.642 | 0.66 m |
| medium | 1 | 2/4 (t = 36.9, 145.7 s) | 1 | 0.466 | 0.554 | 0.23 | 0.50 | 0.451 | 0.95 m |
| large | 5 | 0/5 every run | 0 | 0.141 x5 | 0.000 | 0.00 | 0.00 | 0.469 | 4.95 m |

Mean of world means: **0.267**, against the Phase 1 record of ~0.55
(0.483-0.505 / 0.580 / 0.635-0.646, macOS). **Baseline does not reproduce.**

Large was identical in all five runs (0.141, same extraction, same outcome):
on this machine large behaves as deterministic, unlike the macOS record.

### Why: the flyover extraction gives different answers on Windows

Isolating run `iso-macwp` (large, 1 run): the same Windows build, but with
`victim_location_estimates.csv`, `victim_waypoints_meta.json` and
`map_estimate.png` replaced by the macOS hybrid outputs committed in the team
repo at `48ee484` (the only world for which they survive).

| large_world | Victim est. err | Victims | Overall |
|---|---|---|---|
| Windows preflight | 4.95 m | 0/5 | 0.141 |
| macOS preflight outputs, Windows mission | (Phase 1: 0.30-0.61 m) | 1/5 | **0.642** |
| Phase 1 record (macOS) | 0.30-0.61 m | 1/5 | 0.635-0.646 |

With the macOS waypoints the Windows mission reproduces the Phase 1 large
score to within 0.004. Controller, odometry and physics reproduce; **the
divergence is entirely in the preflight extraction.**

Waypoints vs ground truth, large (mat frame):

| GT victim | Nearest macOS estimate | Nearest Windows estimate |
|---|---|---|
| (8.77, 2.67) | (8.72, 2.10) 0.57 m | (11.89, 2.08) 3.17 m |
| (5.49, -5.76) | (4.71, -5.90) 0.79 m | (6.33, -6.15) 0.93 m |
| (15.92, -7.08) | (16.04, -7.14) 0.13 m | none within 4 m |
| (4.34, 2.74) | (5.06, 2.10) 0.96 m | (5.47, 6.01) 3.46 m |
| (13.10, -1.13) | (13.04, -1.71) 0.58 m | (11.89, 2.08) 3.43 m |

On Windows, `flyover_extraction` falls back to "IMU-assisted victim
estimates" on all three worlds (small: 1 confirmed image detection of 14 raw;
medium: 0 of 5; large: 6 of 51, still IMU-assisted). `prepare_mission` on
medium reports only 27% of samples posed and drops to IMU dead reckoning.

Ruled out so far:
* frame rate / duration: OpenCV FFMPEG backend reports 62.5 fps and the IMU CSV
  has exactly one row per frame (small 4801 rows over 76.8 s), so t = fi/fps is
  correct;
* seeking: `dev/tools/seek_check.py` shows `CAP_PROP_POS_FRAMES` is frame-exact
  on this build at six probe points;
* world files: `large_world.wbt` is byte-identical (same blob) to the team copy;
  the earlier "392 insertions" diff was CRLF line endings.

* container timing: `dev/tools/mp4_timing.py` shows a clean constant 62.5 fps
  (one stts entry, 160/10000 s per sample, no edit offset), so no backend can
  legitimately report another rate.

The wall-map path agrees across platforms (map frame `world_min_x` -8.2820 m on
macOS vs -8.2775 m here); the victim path does not.

### The victim extraction is chaotic in the pixel values

`dev/tools/imu_detect_probe.py --perturb` reruns flyover_extraction's
IMU-assisted victim path with every decoded pixel shifted by one grey level,
about the size of an ordinary cross-platform YUV to BGR rounding difference.
Distance from each ground-truth victim to its nearest estimate:

| World | -1 | as decoded | +1 |
|---|---|---|---|
| small | 1.43 0.75 0.82 5.83 | 0.25 0.77 0.80 5.84 | 0.21 1.25 0.86 5.79 |
| medium | 7.17 1.27 0.52 12.58 | 6.41 1.43 0.62 12.57 | 6.40 0.30 1.05 12.56 |
| large | 3.16 0.98 5.79 1.64 3.45 | 3.17 0.93 5.78 0.77 3.43 | 1.53 0.77 7.32 1.67 4.28 |

(These are the IMU estimates before scout augmentation and submission
selection, which then add further branch points.)

One grey level moves single estimates by up to 1.6 m. The submitted
0.30-0.61 m accuracy was one draw from this distribution, made on one decoder,
with constants fitted on that same draw. It is not reproducible on another
machine, so it cannot have been expected on the judges' machine either: a
concrete mechanism behind the organisers' "lower on generalisation" feedback.

**Consequence for this project.** Re-tuning the extraction until Windows
reproduces the macOS numbers would be fitting to these three videos a second
time (rule 7). The Windows output is taken as the honest baseline. The find
rate has to come from search and final approach that do not need sub-metre
estimates. Any extraction change is judged offline on the mean over the
-1/0/+1 perturbations, not on a single decode.

### Harness changes made to get here (no controller change)

* `run_batch.py` ported to Windows; per-run supervisor JSON and marked report
  copied beside the log; CSV now carries ground-truth victims found, the sim
  time of each find, reports and false reports.
* `telemetry/sitecustomize.py`: Webots on Windows drops `--stdout`, so the dev
  venv tees controller and supervisor output to files when `SAR_TEE_DIR` is set.
* `warm_webots_cache.py`: Webots' own asset download stalled on medium (17
  assets missing) and large (1), hanging world load with no controller started.
  The batch now pre-fetches each world's asset closure and has a 150 s load
  watchdog with retry.
* `make_pristine.py`: per-world flyover snapshot produced by the controller's own
  `_analyse_both` (lifted with `ast`), replacing the old cache that held only
  our own extraction, not the shipped hybrid.

### Per-victim diagnosis (run `diag`, DEBUG_VISION telemetry, 25/11)

Telemetry is behaviour-neutral: `diag` reproduced the baseline exactly (small
0.193, medium 0.466, large 0.141). Odometry tracks agree with the supervisor's
true positions to within centimetres (medium robot2 at its t=71 report: odometry
(15.32, -12.61), truth (15.32, -12.61) in the mat frame). Tracks and closest
approaches from `dev/tools/trajectory_plot.py`; plots in the git-ignored
`dev/eval/runs/diag/*_traj.png`.

| World | Victim | Outcome | Closest approach | Cause |
|---|---|---|---|---|
| small | v1, v2 (SW room) | missed | robot2 0.98 m / 1.05 m, t=123, *through the exterior wall* | robot2 never entered the house: it pressed against the outside of the room's wall for the whole mission |
| small | v2 | missed | robot1 0.63 m, HOME, t=177.7 | robot1 reached the room by partner assist with 3 s left |
| small | v3 (NW room) | missed | robot1 3.43 m, t=45 | could not find the room's door; 40 s timeout, requeued, skipped |
| small | v4 (NE room) | missed | robot1 2.14 m, t=118 | stopped at the room wall; timed out |
| medium | v4 | found t=36.9 | robot2 0.53 m, HOME | homing works when the victim is in view |
| medium | v2 | found t=145.7 | robot2 | partner assist |
| medium | v1 | missed | robot2 0.79 m, DRIVE, t=96.8 | scanned 1.7 m away at t=92 and saw no denim; then drove past with the victim beside it, outside the forward camera's view |
| medium | (false report) | t=71.2 | 3.4 m from v1 | blue object at floor level (median denim height 0.08 m) passed every gate once within 0.72 m |
| medium | v3 | missed | robot1 5.4 m; robot2 2.46 m | robot1 beached on a low obstacle (tilt 13-25 deg) at (13, -14) from t~80 to t~127, 6 m short of its target |
| large | all five | missed | best 2.0 m (robot1, t=12) | both robots beached on sub-lidar rubble within ~10 s of leaving the mat; 7-9 m travelled each |

Causes ranked by victims lost: **cannot get into the room or building
(small: 4)**, **beached on low obstacles (large: 5, medium: 1)**, victim
outside the camera's view while passing or scanning (medium: 1). One false
report from a floor-level decoy. The final-approach oscillation recorded in
Phase 1 does not appear in these runs; small's only HOME episode is the
t=177 one.

## Time curriculum

Rungs: 600, 420, 300, 240, 180 s. Dev runs use `--mission-s N` (dev
supervisor and dev worlds, one line different from the originals each); only
180 s on the originals counts. Three environment copies run one world each in
parallel (`dev/tools/parallel_envs.ps1`); missions are lockstep-deterministic,
so this only changes wall time (~1 sim-s per wall-s with telemetry on).

### Baseline at 600 s (`base600`, controller = phase1-submitted + harness flags)

| World | Victims | Find times (s) | Notes |
|---|---|---|---|
| small | 1/4 | 182.0 | robot2 spent all 600 s outside the house, never through the front door 2 m from its start; robot1 found v2 then sat high-centred at 25 deg (recovery wiggle) for ~400 s |
| medium | 3/4 (+1 false report) | 36.9, 145.7, 444.0 | same first two finds as at 180 s; v3 at 444 s |
| large | 0/5 | - | 18 m travelled in 600 s: robot2 stuck tilted at (0.74, -2.55) from t=10 s; robot1 on the forklift's forks at (4.05, 6.12) from t=30 s |

Even with 600 s, small finds one victim, and 2 s after the real deadline.
More time does not fix entry or beaching: both failures are structural, not a
time budget. (A first attempt to run the three worlds in parallel ran the
mini out of commit memory - 4.5 GB fixed paging file - and crashed the large
and medium controllers; those runs were discarded and rerun sequentially.)

### Final approach (homing), from `dev/tools/homing_episodes.py`

Across `diag` and `base600`: 11 homing episodes, 5 reported (1 of them the
medium floor-decoy false report), 6 aborted after 6.6-12.3 s, ending 2.1-2.7 m
from the nearest marker. Homing starts as far as 3.9 m out at
`APPROACH_SPEED` x `max(0.4, depth/2)` (about 0.27 m/s, falling to 0.11 m/s),
so an approach from 4 m can exceed `APPROACH_TIMEOUT_S` = 12 s and abort
(small 160.8-173.1 s), then re-home after the 8 s cooldown. Small's only find
came 21 s after first sighting.

## Stage 2 experiments

Rung 600 s unless stated. "base600" is the comparison row.

### exp01 - hard walls first (`HARD_WALL_FIRST = True`) - REVERTED

*Hypothesis.* Small's robots cannot enter rooms because the planner treats a
wall cell as merely expensive (cost 30), so a route through a wall beats any
doorway more than ~3 m out of the way. Blocking known wall cells (with the
existing soft fallback when no hard route exists) should route through doors.

*Change.* One flag. Phase 1 had measured it as roughly neutral (0.468 vs 0.472).

| World | base600 | exp01 |
|---|---|---|
| small | 1/4 (182 s) | 0/4 |
| medium | 3/4, 1 false (36.9, 145.7, 444.0) | 2/4, 1 false (36.9, 145.7) |
| large | 0/5 | **1/5 (58.5 s)** |

*Result.* Worse on both small and medium; reverted per the protocol. Large
improved (a different first route avoided the early beaching), which is worth
remembering when the planner is revisited. The telemetry shows
why it could not have worked as hoped: robot2 still spent all 600 s outside,
working sweep points along y = -6, and the front door stayed shut in its map
(see exp02). Small's robot1 found v1/v2's room again but climbed onto the
victim during the marker-offset reposition, tilted to 25 deg, and stayed there
in the HOME state for ~400 s, 0.22 m from v1's marker, without reporting
(tilt recovery pre-empts the reposition deadline).

### exp02 - do not paint the partner robot as a wall

*Hypothesis.* robot2's grid has a wall across small's front doorway, painted
when robot1 drove through it in robot2's lidar view at t~4-10 s. Erasing only
happens along rays shorter than 3 m, and robot2 never returns within 3 m of
the door, so the phantom wall persists. With wall cost 30 per cell and 3x3
painting, crossing it costs as much as a ~9 m detour.

*Change.* Lidar hits within 0.7 m of the partner's last broadcast position
(start position before the first broadcast) are not painted.
