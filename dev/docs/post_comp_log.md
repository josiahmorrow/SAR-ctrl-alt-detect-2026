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

| World | base600 | exp02 |
|---|---|---|
| small | 1/4 (182 s) | **3/4, 0 false (43.6, 92.3, 205.2)** |
| medium | 3/4, 1 false | 2/4, 3 false (34.6, 130.2) |
| large | 0/5 | (running) |

*Result.* Small: robot2 now walks straight in through the front door and finds
v1 and v3; robot1 finds v4. v2 missed with robot2 0.28 m from its marker
right after reporting v1 1.47 m away: the homing block is a 1.5 m circle
around the *robot's own position* (exp04).

Medium regressed by the protocol's measure, through two mechanisms exp02 does
not touch, exposed by the changed paths:
1. robot1 reported the floor decoy at true (15.39, -12.95) that robot2 had
   reported at (15.31, -12.61). Its odometry read (16.20, -14.23): tilt and
   wheel slip had moved it ~1.5 m off, so "already reported within 1 m" missed.
2. robot2 reported v3 from 1.01 m (true (11.80, -19.38) vs marker
   (11.80, -20.39)): the marker-offset problem from the brief.

**Kept, contrary to the revert rule, and flagged to Joe.** The argument is
general (the partner is never a wall, in any world) and the medium losses
trace to separate, identified causes.

Large under exp02: 0/5 (unchanged from base600).

### exp03 - never climb onto a confirmed victim - REVERTED

*Hypothesis.* During the marker-offset reposition the IR low-obstacle guard is
disabled, so robots drive onto the victim's body, tilt past 25 deg and sit in
the HOME state for the rest of the mission (exp01 small: 400 s, 0.22 m from
v1's marker, never reported); tilting also corrupts odometry (exp02 medium).
*Change.* (a) IR contact while repositioning counts as a stall: report from
there; (b) while homing, IR contact with denim within 1.0 m counts as "close
enough" for the confirmation streak, and the creep stops; (c) tilt while
repositioning ends the approach and reports.

| World | exp02 (base) | exp03 |
|---|---|---|
| small | 3/4 (43.6, 92.3, 205.2) | 2/4 (43.4, 199.9), 314 tilt events |
| medium | 2/4, 3 false | not run (small regressed) |

*Result.* Reverted. robot2 reported v1 at 43.4 s as before, then stayed in
v1/v2's room for the rest of the mission (candidate in view on 1,023 telemetry
ticks, blocked by "near prior report"), so it never went on to v3 as it did in
exp02. Tilt events rose, not fell.

*Method note.* Small changes reroute the robots and the outcome moves by a
victim either way for reasons unrelated to the change (exp02 medium, exp03
small). Each world is deterministic, but one run per world measures one
trajectory, not the controller. Proposed: a dev-only "seed" (start-of-mission
delay of 0/50/100 ms, env-set, default off) and judging each change on the
mean over three seeds per world. That triples cost, so it needs the parallel
runs, which need the bigger paging file.

### exp04 - "already reported" judged by the candidate's position - REVERTED

*Hypothesis.* exp02 small missed v2 with robot2 0.28 m from its marker
because the homing block is a 1.5 m circle around the robot's own position and
v1 had just been reported 1.47 m away. Comparing the camera's estimate of the
candidate body (depth + bearing) with the estimated bodies already reported,
at 1.0 m separation, should keep neighbours findable.

| World | exp02 (base) | exp04 |
|---|---|---|
| small | 3/4, 0 false | 1/4, 1 false (43.6) |
| medium | 2/4, 3 false | not run |

*Result.* Reverted. robot2 reported v1, then homed on v1's body seen from
another side (its denim centroid more than 1.0 m from the first estimate: a
lying body is ~1.7 m long), was sent toward v2's CSV estimate by the
marker-offset reposition, stalled after 6 s at a point 0.75 m from v1's marker
and reported "from wherever it got to": false. Distinguishing two adjacent
lying bodies by blob centroid is not reliable at this separation, and the
reposition's report-on-stall fallback is a false-report generator.

### Harness: trajectory seeds

`SAR_SEED` (dev only; unset = 0 = submitted behaviour) lengthens the startup
spin by 50 ms per step. `run_batch.py --seeds 0,1,2` runs one mission per
seed. Neutral at seed 0 by construction (3.5 + 0.0 s).

### exp02 across seeds (`exp02s`; seed 0 = the original exp02 run)

| World | seed 0 | seed 1 | seed 2 | mean found | mean false |
|---|---|---|---|---|---|
| small | 3/4, 0 false (43.6, 92.3, 205.2) | 2/4, 1 false by 450 s (physics stall, aborted) | 2/4, 0 false (33.9, 492.6) | 2.33 | 0.33 |
| medium | 2/4, 3 false | 1/4, 1 false | 2/4, 4 false | 1.67 | 2.67 |

Every medium run reports the same decoy: true robot position (15.24-15.40,
-11.84 to -12.95), next to two blue `PlasticCrate`s (0.6 x 0.4 x 0.3 m) and two
`PlatformCart`s. Median denim height 0.08 m and growing apparent area as the
robot closes in let it through every gate. The second false-report site is
1.0-1.3 m north of v3's marker, where pipes and boxes stop the robot short of
the body (marker-offset problem). base600 medium seeds 1 and 2 are running
for a fair comparison (mask radius 0 = Phase 1 behaviour).

A physics slowdown (~0.07 sim-s per wall-s) hit small seed 1 at t=450 s;
the run was stopped. It costs wall time only, never sim outcome.

### Fair comparison on medium (`base600s`: Phase 1 behaviour, seeds 1 and 2)

| medium @600 s | seed 0 | seed 1 | seed 2 | mean found | mean false |
|---|---|---|---|---|---|
| base600 (no partner mask) | 3/4, 1 false | 1/4, 1 false | 1/4, 2 false | 1.67 | 1.33 |
| exp02 (partner mask) | 2/4, 3 false | 1/4, 1 false | 2/4, 4 false | 1.67 | 2.67 |

Neutral on medium finds; the extra false reports are more visits to the crate
decoy. Small: base600 seed 0 found 1, exp02 never fewer than 2 in three seeds.
exp02 stays. The base600 seed-0 medium result (3/4) was the lucky draw.

### Decoy inspection (`snapM`: exp02 + texture/snapshot telemetry, medium seed 0)

Telemetry-only patch (`SAR_SNAP_DIR` saves the camera frame and denim blob
mask once per second while a candidate is in view; telemetry line gains
`tex_v`/`tex_s`, the std of HSV value and saturation inside the denim blob).
Neutrality: identical to exp02 seed 0 through t=165 s (reports 34.6 TRUE,
~78 false, 130.2 TRUE, ~165 false); diverges after that (no 328 s false
report), 2/4 found, 2 false.

The camera frame at the 78.6 s false report shows the decoy is the
**PlatformCart**: its blue plastic rim is the "denim" blob (a thin band on
the floor, beige deck inside). Per-site candidate texture:

| Site | frames | tex_s median [range] | tex_v median |
|---|---|---|---|
| real victim, robot2 t=23-31 | 16 | 15.6 [15.2-16.4] | 12.4 |
| real victim, robot2 t=114-131 | 33 | 15.0 [12.7-15.9] | 9.8 |
| real victim (not reported), robot2 t=274-283 | 16 | 23.9 [20.9-27.0] | 14.4 |
| cart decoy, robot2 t=79-82 | 4 | 6.0 [4.9-6.3] | 10.8 |
| cart/crate site, robot1 t=140-159 | 15 | 4.4-7.9 | 4.8-13.1 |

Saturation spread separates them: printed denim texture varies in
saturation (weave, fading, folds), flat-coloured moulded plastic varies only
in shading (value). Gap 7.9 to 12.7 on medium. Before using it: collect the
same on small and large (`snapS`) and medium seeds 1 and 2 (`snapM2`).

### Texture is not a victim invariant (`snapS`, `thickS`, `thickM`)

Small changes the picture. On small the largest denim-hue blob is often the
victim's plain navy **shirt**, not the jeans (the faded jeans fall below the
mask's saturation floor), and a plain shirt is as flat as the cart rim:
victim frames on small read tex_s 3.2-21 (median 6.7). A texture gate would
have rejected the 43.6 s true find. Rejected.

Added telemetry `thick` (p75 over blob columns of run length x column depth
/ focal, metres; distance-invariant). Cart rim on medium: 0.006-0.032 m in
every frame. Victims: mostly 0.13-0.50 m, but close-range and climbing
frames drop to 0.002-0.03 m on small, and the v3 approach on medium reads
0.032-0.078. The conjunction thin-and-flat still catches some real-victim
frames. Both cues are fitted to one object on one world (rule 7); the cart
costs confidence, not finds. Parked.

### Duplicate reports are scored False; odometry runaway causes them

The supervisor skips victims already found when it checks a report, so a
second report at a found victim is a **False** verdict. Small seed 1
(`thickS` run 2): 3/4 found, both False reports were robot1 parked 0.27 m and
0.6 m from victims robot2 had already found. Our own `_already_reported_nearby`
check failed because odometry was wrong: robot2 reported victim 2 at odometry
(2.87, -41.87), 40 m from the truth, after 160 s of recovery wiggle while
high-centred at 25 deg on a victim's body (588 tilt lines). Encoder travel
is integrated whatever the chassis attitude, so spinning wheels on a beached
robot walk the estimate away.

### Run-to-run determinism

Same seed, same inputs: `snapS` and `thickS` small seed 0 are identical (1/4,
99.4 m, 494 tilt lines). Medium seed 0 in `thickM` matches the original exp02
run report for report up to 328 s, but `snapM` diverged after 165 s. So outcomes are
near-deterministic with occasional chaotic divergence; means over seeds stay
the measure. `detchk` reruns the committed 8d2ced9 controller on small seed 0
(exp02 there: 3/4) to separate code effects from this.

### Medium seed 0 ground truth (`thickM` run 1)

Cart reports at true (2.41, 3.39) and (2.49, 3.05). v3 marker (-1.10, -4.39):
report at (-1.10, -3.38) = 1.01 m, False; 40 s later at (-1.27, -3.48) =
0.93 m, True. v1 (4.11, 0.45) never found.

### detchk result (after the 18:59 reboot)

Commit 8d2ced9 (no telemetry patch) on small seed 0: 1/4, 99.4 m, 494 tilt
lines, found 43.6 s. Identical to `snapS` and `thickS` seed 0. The telemetry
patches are behaviour-neutral; the original exp02 small seed-0 3/4 was an
earlier sim state that no longer reproduces. Current exp02 small reference:
seeds 0/1/2 = 1, 3, 2 found (mean 2.0).

### exp05: freeze encoder translation while tilted (abandoned after 1 run)

Hypothesis: encoder travel while high-centred is not body travel; freezing it
keeps `_already_reported_nearby` honest. `ODOM_TILT_FREEZE`: no translation
while tilt > 25 deg, or > 11 deg sustained 0.35 s.

Small seed 0: 2/4 found (43.6, 69.6), but **60 False reports**. robot2 sat
pressed against a victim, untilted, camera frame frozen (depth 0.61 m
constant), while odometry ran 0.3 m/s; every metre of phantom travel
cleared the duplicate check and it reported again. Wheel slip without tilt
is the general case. Remaining seeds not run.

### exp05b: exp05 + lidar slip rejection

Every 0.5 s, re-project the previous scan's endpoints under two hypotheses
(encoder pose vs. no translation) and compare medians against the current
scan. If "no translation" fits at least twice as well, undo the window's
encoder translation. Ambiguous geometry keeps the encoders. Heading stays
from the compass. ~55 lines.

| world @600 s | s0 | s1 | s2 | mean found | mean false |
|---|---|---|---|---|---|
| small exp05b | 1/4, 0 F | 2/4, 0 F | 1/4, 0 F | 1.33 | 0 |
| small exp02 (current reference) | 1/4, 0 F | 3/4, 2 F | 2/4, 0 F | 2.0 | 0.67 |
| medium exp05b | 1/4, 1 F | 1/4, 1 F | 2/4, 1 F | 1.33 | 1.0 |
| medium exp02 (thickM s0, s1; exp02s s2) | 3/4, 3 F | 1/4, 1 F | 2/4, 4 F | 2.0 | 2.67 |

Slip undos are rare (0-13 per robot per run) and look genuine (stay residual
0.01-0.05 m vs move 0.08-0.27 m). False reports fall; finds fall too, but the
finds are dominated by something else: in 4 of 5 runs a robot is
permanently beached (tilt lines 2252, 13471, 14908). In small s0, s1 and s2
robot2 beaches immediately after its first find: the marker-offset
reposition drives from 0.75 m straight at the CSV estimate 1.2 m away,
across the victim's body, reports from on top of it, and never comes down.
That is exp06.

### exp06: end the reposition when the front IR sees the body (inert)

`REPO_BODY_IR_M = 0.15`: during a reposition, front IR closer than this counts
as a stall (report from here). Stacked on exp05b.

| world @600 s | s0 | s1 | s2 |
|---|---|---|---|
| small | 1/4, 0 F (1513 tilt lines) | 2/4 by 450 s, 0 F (physics stall, stopped) | 1/4, 0 F |
| medium | 1/4, 1 F | 1/4, 1 F | 1/4, 1 F |

Report times and tilt counts match exp05b run for run (small s1 2252 tilt
lines in both): the stop never fired. TILT accelerometer telemetry shows why:
the climb starts during the in-place turn *after* the report (small s2
robot2 t=37.4, cmd=(3.1, -5.5)), where the front IR sees nothing. Inert;
removed when the next experiment is assembled.

### exp07: back out after a report; victims and low obstacles stay in the map

One hypothesis: robots beach on the victim they have just reported. Three
mechanisms, all aimed at that body:
- after a report, reverse straight out for 1.0 s (about 0.25 m) before any turn;
- paint the reported body's footprint into a persistent low-obstacle layer
  (depth and blob width from the confirming frame: broadside = wide and
  shallow, end-on = narrow and deep);
- IR and tilt-guard low obstacles go into the same layer, so the lidar's
  free-space erase (its plane passes over them) can no longer delete them.
Stacked on exp05b + exp06 (exp06 inert, left in as tested). ~80 lines.

| world @600 s | s0 | s1 | s2 | mean found | mean false | tilt lines |
|---|---|---|---|---|---|---|
| small exp07 | 3/4, 3 F | 2/4, 1 F | 3/4, 0 F | **2.67** | 1.33 | 52, 54, 41 |
| small exp05b | 1/4, 0 F | 2/4, 0 F | 1/4, 0 F | 1.33 | 0 | 6, 2252, 13471 |
| small exp02 | 1/4, 0 F | 3/4, 2 F | 2/4, 0 F | 2.0 | 0.67 | 494, 781, 21 |
| medium exp07 | 1/4, 1 F | 2/4, 1 F | 2/4, 2 F | 1.67 | 1.33 | 6, 13, 7 |
| medium exp05b | 1/4, 1 F | 1/4, 1 F | 2/4, 1 F | 1.33 | 1.0 | 8, 14908, 9 |
| medium exp02 | 3/4, 3 F | 1/4, 1 F | 2/4, 4 F | 2.0 | 2.67 | 49, 11, - |

Beaching on reported victims is gone (tilt lines in the tens in all six
runs). Totals over 6 runs: exp07 13 found / 8 False, exp02 12 / 10. Small
improves; medium's mean is 0.33 lower than exp02's, inside the noise
(exp02 medium s0's 3/4 has not reproduced since), so this is flagged rather
than reverted. Kept.

Remaining false reports on small are duplicates at found victims (v1 and v2
lie 1.4 m apart; partner positions drift apart by more than the 1.0 m
duplicate radius). Medium robot2 now spends most of the run trapped on
sweep waypoints along x = 18 (timeouts, "written off" regions).

### exp07 on large (`exp07L`, seeds 0, 1 @600 s)

0/5 both runs, 21.4 m and 20.8 m travelled. Both robots end **inverted**
(tilt 131 deg) inside 40 s: robot2 at t=5.3 s near the start (accelerometer
spike 59 m/s^2 on contact, front IR had read 0.22 m half a second earlier),
robot1 at t=35 s (spike 199 m/s^2, IR 0.21 m). Contact with large's rubble at
cruise speed produces solver-scale impulses that flip the chassis; nothing
after that can help. Next: exp09, slow down when the front IR sees a low
obstacle (closing speed is what sets the impulse).

### exp08 (queued): second look at a missed flyover estimate

Medium exp07 s0: robot2 reached 1.5 m from v1's estimate, scanned, saw only
a sliver of denim past the platform cart, retired the waypoint and never
came back. exp08 queues one more look from the opposite side (1.2 m from
the estimate, on a free cell) when a scan at an estimate finds nothing.

### exp08 result (kept, commit 6a1afaa)

| world @600 s | s0 | s1 | s2 | mean found | mean false |
|---|---|---|---|---|---|
| small exp08 | 4/4, 2 F | 2/4, 1 F | 2/4, 1 F | 2.67 | 1.33 |
| medium exp08 | 1/4, 3 F | 3/4, 1 F | 2/4, 1 F | 2.0 | 1.67 |

First 4/4 on any world (small s0; the second look fired at (7.8, 6.5) and
that victim was then found). Totals 14/24 vs exp07 13/24. Medium s0's false
reports are robot1 at the true v3 and v2 bodies but 1.20 m and 2.06 m from
their scored positions (obstacles keep it off the body), plus the cart.

### exp09: IR speed governor (large only, not kept)

Slow toward front-IR low obstacles. Large s0, s1: 0/5, 21.4 m and 11.8 m.
The flips come with no IR warning (IR inf half a second before, robot thrown
0.5 m in one tick, then inverted). Not a contact-speed problem.

### Large diagnostic: the second Floor node (`diagL`, DEV-ONLY world copy)

`large_world.wbt` has two `Floor` nodes at z = 0 (`floor(1)` 16 x 16.4 at
(2.5, -4.3), `floor(2)` 7 x 18 at (-9, -4)); medium has one. Every large
flip so far happens on open floor with nothing within 1.2 m in the world
file. A diagnostic copy of the dev world without `floor(2)` (git-excluded;
the judged world is untouched and the dev world was restored from backup):

| large @600 s (no floor(2)) | found | false | tilt lines | distance |
|---|---|---|---|---|
| seed 0, exp08 controller | **3/5** (43.4, 236.0, 401.2) | 0 | 10 | 159.8 m |
| seed 1, exp08 controller | **2/5** (63.0, 141.0) | 1 | 5 | 173.7 m |

On the untouched world the same controller makes 0/5 with both robots
inverted within 40 s. The large failure is a world/physics defect, not a
navigation one; it cannot be fixed under rule 2. Raised with Joe.

### exp10: known-body gate on the painted footprint (rejected after 1 run)

Block homing/reporting when the denim's floor point lands on a painted victim
footprint (own, or a 1.0 m disc round the partner's report). Small s0: 1/4
(exp08: 4/4). No `known-body` block appeared in telemetry, but the report-time
gate has no telemetry, and robot2 never reported v2 (1.4 m from v1, inside
v1's 1.3 m-deep footprint). Footprints are too coarse to separate neighbours.
Stopped after one run.

### Why exp08 misses victims (trajectory_plot, odometry-based)

- small s2, s3: v2 missed although robot2 passed 0.50 m from it while homing
  on v1 (1.4 m away). The homing gate refuses any candidate while the robot is
  within `HOME_BLOCK_R_M` = 1.5 m of a previous report *position*; the
  victims are closer together than that. exp11 targets this.
- medium: v1 (next to the platform carts) and v3 (behind pipes/boxes) are the
  usual misses; robots pass 0.65-1.0 m from v1 and ~1 m from v3 without a
  candidate (`denim-blob-small` dominates the gate reasons there).

### exp11: identify victims by body point, not robot position (rejected)

Known-victim checks (homing gate, report gate) compare the denim blob's floor
point (bearing from image column, range from depth camera) with recorded body
points (own and partner's, broadcast), radius 0.7 m; robot-position radius
cut to 0.5 m.

| world @600 s | s0 | s1 | s2 | mean found | mean false |
|---|---|---|---|---|---|
| small exp11 | 2/4, 0 F | 2/4, 3 F | 3/4, 2 F | 2.33 | 1.67 |
| small exp08 | 4/4, 2 F | 2/4, 1 F | 2/4, 1 F | 2.67 | 1.33 |
| medium exp11 | 1/4, 3 F | 3/4, 2 F | 2/4, 1 F | 2.0 | 2.0 |
| medium exp08 | 1/4, 3 F | 3/4, 1 F | 2/4, 1 F | 2.0 | 1.67 |

No gain in finds, more repeat reports (body points of one victim seen from
different sides differ by more than 0.7 m). Reverted.

### exp12 (running): marker-offset reposition off

Every beaching on small since exp07 starts inside the reposition (drive from
0.75 m at the CSV estimate, across the body). Test: `REPOSITION_MAX_M = 0`.

### exp12 result: reposition off (rejected after 2 runs)

Small s0 2/4 (5746 tilt lines), s1 1/4 (6893) against exp08's 4/4 and 2/4
on the same seeds. Robots still beach without the reposition; it was not
the cause. Stopped to free the machine for the approved rewrite.

## Navigation rewrite (approved by Joe 26/11/2026 08:22)

### nav1: log-odds occupancy + hard inflated planner

`make_nav1.py` on exp08 (~120 lines):
- log-odds grid per scan: pass -0.4 along each sampled beam (0.25 m to 0.12 m
  short of the return, up to 5 m), hit +0.9 at returns under 5 m, clamp
  [-4, 4], wall while > 1.0; each cell counted once per scan; partner mask
  kept;
- planner first searches with walls and the persistent low-obstacle layer
  inflated by the chassis radius (0.18 m) as impassable (start neighbourhood
  exempt, a blocked goal moved to the nearest free cell within 1 m), falling
  back to the soft-cost search; `HARD_WALL_FIRST = True` again (exp01 failed
  on the binary map, where noise sealed doors).

Offline test (`_scratch/test_nav1.py`, synthetic room, 90 of 360 beams): a
body blocking a doorway for 10 scans clears after 7-10 clear scans (~3 s);
real walls keep 97-100% cell coverage.

### nav1 result

| world @600 s | s0 | s1 | s2 | mean found | mean false | coverage |
|---|---|---|---|---|---|---|
| small nav1 | 2/4, 0 F | 2/4, 0 F | 1/4, 0 F | 1.67 | 0 | 64, 61, 99% |
| small exp08 | 4/4, 2 F | 2/4, 1 F | 2/4, 1 F | 2.67 | 1.33 | 51, 37, ?% |
| medium nav1 | 2/4, 2 F | 3/4, 1 F | 3/4, 2 F | **2.67** | 1.67 | 68, 72, 71% |
| medium exp08 | 1/4, 3 F | 3/4, 1 F | 2/4, 1 F | 2.0 | 1.67 | |

Coverage rises sharply (small up to 99%, medium ~70%) and medium finds rise;
small finds fall. Small s2 is the telling run: 99% lidar coverage, 1/4
found, and trajectory_plot shows robots passing v1, v3 and v4 at about 1 m
in DRIVE with no candidate frame; robot2 never saw denim at all. The camera
looks along the route, so lidar coverage is not camera coverage. nav2 adds
camera-coverage look-arounds. Not kept on its own (small regresses).

### nav2: camera-coverage look-arounds (rejected)

On nav1, keep a grid of cells the camera has seen (FOV, 3 m, cut at the
lidar return); turn on the spot (6.5 s) when more than half the free floor
within 2.5 m is unseen, at most every 10 s.

Small 0/4, 2/4, 0/4; medium s0 1/4 with 4 False. Stopped. In the small runs
robot2's sequence at v1 is unchanged: second look from the far side, then the
marker-offset reposition drives across the body, reports False from on top
of it (4.17, -2.9 odometry vs the usual 3.35, -3.3) and beaches for the rest
of the run. Look-arounds did not cause that, but they did not offset it either.

### Two missions in parallel (memory permitting)

Commit charge with one Webots: 33.7 GB limit, 7.1 GB free; Webots ~3.8 GB.
`_scratch/par2.ps1` runs small in sar-env and medium in sar-env2 at the same
time (`SAR_PARALLEL=1` so neither batch kills the other's Webots; tags
`<tag>S` / `<tag>M`). First use: exp13.

### exp13: gravity-guided recovery from high-centring (rejected)

Replace the symmetric fwd/rev wiggle: stop 0.6 s so the accelerometer reads
gravity alone, then push 2 s toward the low side of the chassis (every third
push the other way). Small 2/4, 2/4; medium 2/4, 2/4 (seeds 0, 1; seed 2 lost
to the reboot). A beached robot made 160 pushes at a steady 33 deg nose-down
and never came off: the chassis is high-centred on the body with the drive
wheels unloaded, so no wheel command frees it. Prevention is the only lever
(nav3).

### Paging file and parallel runs (26/11 11:18 reboot)

Paging file set to System managed (approved by Joe). After the reboot,
`_scratch/parN.ps1` schedules one-run jobs over three environment copies
(tags `<exp>_<W><seed>`, aggregated by `_scratch/agg.py`); the page file grows
on demand (5 GB within minutes of three missions). Missions run at about half
real time with three in parallel: ~20 min each, three at once.

### nav3: depth-camera low obstacles (first version withdrawn)

Project depth pixels 7-30 cm above the floor (planar depth model, level
chassis, <1.8 m) into the persistent low-obstacle layer. First three runs:
small s0 0/4, small s1 3/4, medium s0 3/4. Small s0 shows the flaw: anything
that moves (the partner robot beside the start, which is painted before its
first position broadcast) stays an obstacle for good; robot2 spent the run
"trapped" 4.3 m from v1's estimate. Withdrawn.

### nav3b: the same with free-space clearing

Depth layer kept separately as log-odds per cell: +1.0 per scan with enough
returns in the band, -0.6 per scan where the camera sees floor (|z| < 3 cm),
obstacle while > 1.5. The partner or an opened door is cleared once the
camera sees floor there again. Running: 6 seeds per world (`nav3b_*`).

Result, seeds 0-5 at 600 s: small 2.17/4 (sd 0.69, false 0.17), medium
1.67/4 (sd 0.75, false 1.67) against exp08 seeds 0-2 small 2.67 (1.33 F),
medium 2.00 (1.67 F). Fewer false reports on small, fewer finds on both.
**Rejected.**

### Failure census, 26/11 (offline, ground truth used for evaluation only)

Three offline tools in `_scratch/` (not part of the controller):
`beach.py` (per robot: time tilted, beached/flipped and when), `falses.py`
(every report: verdict, true distance to the nearest victim, dup/far/phantom),
`repfeat.py` (vision features in the 2 s before each report, by verdict).
Over exp07, exp08, exp13, nav1 and nav3b runs:

- The flyover estimates are good (small 0.66 m mean error) and the video
  waypoints put robots beside the victims. The losses are local.
- **Beaching dominates small.** nav3b small: 7 of 12 robot-runs end beached;
  exp08 small 2 of 6. Many are flips (median tail tilt 76-131 deg), the
  rest sit at 13-25 deg where the tilt guard holds a reverse that never
  frees them. The typical sequence (exp08 small s1 robot2): report between
  two bodies 1.4 m apart, a 0.2 m straight reverse, then the next leg's
  turn presses on the body for 6 s (front IR 0.17-0.45 m) until it mounts.
- **Medium phantoms.** One spot, a platform cart with plastic crates about
  3.4 m from victim1, draws a false report in most medium runs. Its frames
  have blob saturation std <= 6.5 and vertical thickness <= 0.052 m. No true
  report has both. (World geometry was read offline to identify the object.)
- **End-on reports just outside 1.0 m.** Medium victim3 is reported from
  1.13-1.20 m with a 0.34-0.53 m wide blob: the body is seen feet first and
  the scored point lies beyond the visible denim.
- **Duplicates** of a robot's own earlier report after odometry drift
  (exp08 small s0 robot2 at 216 s and 309 s).
- The "no matching recording" preflight line on `*_dev` worlds is harmless:
  `run_batch.py` restores the pristine per-world flyover outputs before each
  run. The untouched worlds match by name.

Queued from this (each on exp08, six seeds per world, `jobs4.txt`):
exp16 back out along the inbound trail after a report; exp17 flat, thin,
untextured close-range reject; exp18 flank end-on bodies before reporting;
exp15 own-report dedup on body position; exp14 partner false-positive zone
2.5 -> 0.8 m. Also an exp08 baseline for seeds 3-5 (`exp08b`).

### Baseline seeds 3-5 (exp08b)

exp08 on seeds 3-5: small 3, 1, 2 (0 F); medium 1, 2 (2.5 F); medium s5 hit
the wall-clock timeout at 545 s sim (physics slowdown) and is void, to be
rerun. Six-seed exp08 baseline: **small 2.33/4 (0.67 F), medium 1.80/4 over
five seeds (2.0 F)**. The 3-seed exp08 figure (2.67 small) was a lucky draw.

### exp16: back out along the inbound trail after a report (kept)

After each report, reverse 0.7 m along the robot's own recorded track (poses
every 5 cm while level), steering the rear onto it; 6 s cap; tilt guard keeps
priority. Offline kinematic check: retraces to within 0.07 m.
Seeds 0-5: small 2.33/4 (sd 0.75, 0.33 F), medium 2.17/4 (sd 0.37, 1.83 F)
against the baseline small 2.33 (0.67 F), medium 1.80 (2.0 F). No regression,
medium +0.37 (about one standard error), half the small false reports.
**Kept.** It does not stop beaching: 5 of 12 small robot-runs still end
beached or flipped, many away from any report.

### exp17: flat/thin/untextured close-range reject (rejected)

Rejects the crate seen in earlier runs, but the robots then report the same
cart from other angles where the blob's saturation std is 7.9-8.0 (thickness
0.02-0.03 m), just above the 6.8 cut, and new phantoms appear 2.3-4.5 m from
any victim. Seeds 0-5 small 2.17 (0.83 F); medium seeds 0-4 1.60 (1.8 F).
Regresses both. The threshold was fitted to these worlds (rule 7) and did
not hold even on them. **Rejected.** No single vision feature separates the
cart from true reports: thickness alone overlaps (true reports down to
0.022 m).

exp17 medium s5 completed later: 2/4; six-seed medium 1.67 (1.5 F). Verdict
unchanged.

### exp18: flank end-on bodies before reporting (rejected, mostly inert)

If the locked denim blob is narrower than 0.45 m (body seen end-on), step
0.6 m sideways to the side with more lidar room, drive forward to abreast
the far end of the body, and report there. Seeds 0-5 small 1.83/4 (0.5 F),
medium seeds 0-4 2.00/4 (2.2 F). It fired once in eleven runs (medium s1,
which then found 4/4). The measured width at lock is usually 0.55-0.67 m
even where the report lands 1.3 m away, so the end-on test rarely holds.
The small drop (2.33 -> 1.83) comes from runs where it never fired: chaotic
divergence, not the change. Under the protocol, small regressed:
**rejected.**

What the far reports actually are (exp18 M3 robot1, t=133): denim locked
at 0.74 m, width 0.67, then "repositioning toward estimate (10.81,-24.86)".
That flyover estimate lies away from the body in view, so the robot drives
off the victim and reports 1.32 m from it. The same happens for medium
victim2 in exp08b M3/M4, exp16 M0/M3/M4 and exp18 M3/M4. This is exp19.

exp18 medium s5: 2/4; six-seed medium 2.00 (2.0 F). Verdict unchanged.

### exp15: own-report dedup on body position (rejected)

Own reports store a body estimate (pose + camera bearing + denim depth +
0.2/0.6 m by lie); a candidate is a duplicate only within 1.0 m of an own
body; partner reports keep the 1.5 m robot-position gate. Seeds 0-5: small
2.17/4 (1.17 F), medium 1.83/4 (1.67 F). It does what it was meant to in one
case (small s4: robot2 reported v2 then the adjacent v1 13 s later, both
true), but the body estimate scatters by more than 1 m between views of the
same victim: small s0 re-reported v1 four times, s2 twice, s5 once (all
"dup", 0.28-0.72 m from v1). Small false reports up from 0.67 to 1.17, finds
down. **Rejected.** Adjacent victims need a dedup cue that is stable across
views (or a second report only when two distinct bodies are seen at once).

### exp14: partner false-positive zone 2.5 -> 0.8 m (rejected)

Seeds 0-5 small 2.00/4 (0.33 F); medium seeds 0-4 1.80/4 (2.0 F). Medium
equal to baseline, small down 0.33. **Rejected.** The partner zone is not
what costs finds in these runs.

### Five-experiment summary (exp14-exp18), 26/11 20:50

Baseline exp08, 6 seeds at 600 s: small 2.33/4, medium 1.80/4.
Kept: exp16 (trail back-out). Rejected: exp14, exp15, exp17, exp18.
Consecutive experiments without improvement since exp16: 4.
Queued (jobs5, on exp08+exp16): exp08 medium s5 rerun, exp19 (reposition
only to an estimate that matches the body in view), exp20 (paint every body
seen within 2 m as a low obstacle).

### Baseline medium s5 rerun (exp08r)

2/4 (1 F). Six-seed exp08 baseline: **small 2.33/4 (0.67 F), medium
1.83/4 (1.83 F)**.

### exp19 (on exp08+exp16): reposition only to an estimate matching the body in view (rejected)

Before driving from the locked denim to the nearest flyover estimate, check
that the estimate lies within 1.0 m of the body's centroid (pose + depth +
0.3 m along the camera axis); otherwise report from where we are.
Seeds 0-5 small 2.00/4 (0.33 F), medium 2.17/4 (1.50 F) against the kept
stack exp16 small 2.33 (0.33 F), medium 2.17 (1.83 F).
Medium: the 1.2-2.0 m "far" reports are gone (zero in six runs, against
1-2 per run before) and false reports drop 1.83 -> 1.50; finds unchanged,
because the remaining medium misses are victim1 (never reached) and the
cart phantom. Small regresses: on small the estimate the robot used to
drive to belongs to the ADJACENT victim (v1 estimate between v1 and v2), so
the old reposition accidentally reported v1 from v2 and then v2 later;
reporting from the body finds v2 and then the near-prior-report gate hides
v1 for the rest of the run (s2: 1/4). **Rejected under the protocol**, but
the medium mechanism is real. exp21 pairs it with a fix for the adjacency
gate (one hypothesis: report from the body, and let an unclaimed flyover
estimate vouch for a second victim next to a reported one).

### exp20 (on exp08+exp16): every body seen within 2 m is a low obstacle (rejected, borderline)

Every 0.5 s while not homing, a candidate passing the vision gate at a
finite depth <= 2 m (level chassis) has its footprint (camera bearing,
measured depth, width-dependent extent) painted into the persistent low
layer, and the route is replanned.
Seeds 0-5 small 2.50/4 (0.17 F, s2 4/4), medium 2.00/4 (1.50 F) against the
kept stack small 2.33 (0.33 F), medium 2.17 (1.83 F). Total finds equal
(27/48 both), false reports 10 vs 13, small up, medium down one victim over
six runs. Beached robot-runs: small 4/12 (was 5/12), medium 3/12 (was 2/12).
Under the protocol medium regressed: **rejected**, but by one victim in six
runs, inside run-to-run noise; worth retesting combined with exp21 later.
No-improvement streak since exp16: 6.

### exp21 (exp19 + estimate-backed second victim) (rejected)

When a homing candidate is within 1.5 m of a prior report, allow it if the
body in view (pose + depth + 0.3 m) is within 1.2 m of a flyover estimate
not claimed by an own report; each own report claims the estimate nearest
its body. Seeds 0-5 small 2.00/4 (1.00 F), medium 2.17/4 (1.50 F).
Medium identical to exp19. Small: the v1 estimate (2.94,-3.12) lies within
1.2 m of BOTH small bodies, so after v2 is reported the robot re-homes on
v2 under v1's estimate and re-reports it (dup at 0.55-0.64 m in s0, s2,
s5), or loops homing/aborting on it (hundreds of attempts in s1, s2, s4).
A 1.2 m match radius cannot separate two bodies 1.4 m apart with estimates
of ~0.7 m error. **Rejected.** No-improvement streak: 7.

exp22 (exp21 + exp20) was cancelled after starting, since it carries the
same defect; its three started runs are orphaned and not scored. Replaced
by exp23 = kept stack + exp19 + exp20 (the two borderline changes together:
exp20 helped small, exp19 removed medium's far reports).

### exp23 = exp08 + exp16 + exp19 + exp20 (kept)

The two borderline changes together: report from the body unless the
flyover estimate matches it (exp19), and paint every body seen within 2 m
as a low obstacle (exp20). Seeds 0-5 at 600 s:

| | small found (F/run) | medium found (F/run) |
|---|---|---|
| baseline exp08 | 2.33 (0.67) | 1.83 (1.83) |
| kept exp16 | 2.33 (0.33) | 2.17 (1.83) |
| exp19 alone | 2.00 (0.33) | 2.17 (1.50) |
| exp20 alone | 2.50 (0.17) | 2.00 (1.50) |
| **exp23** | **2.67 (0.17), sd 0.47** | **2.67 (0.83), sd 0.47** |

Both worlds up; no seed below 2/4. The pair works where neither did alone:
reporting from the body no longer drives the robot off v2 onto the
adjacent v1 estimate, and painting the seen-but-blocked v1 keeps the next
leg from climbing it, so v1 is found later from another side (small s2,
s3; medium victim1 now found in s1 and s2). Remaining falses: the medium
cart phantom (4 of 6 runs) and two 1.02-1.15 m near misses. Beached
robot-runs: small 3/12, medium 2/12.
**Kept** (commit 8eb581d). No-improvement streak reset to 0.
Still below target: 67% on both worlds, no world at 4/4 twice.

Next: exp24 (escalate a failed straight reverse in the 11-25 deg band to
the rocking recovery), on exp23.

### exp24 (on exp23): escalate a failed straight reverse to rocking (kept)

In the 11-25 deg band the tilt guard held a straight reverse for as long as
the chassis stayed tilted. After 4 s without freeing, switch to the
fwd/back/turn pulse cycle used above 25 deg. Seeds 0-5 small 2.67/4
(0.17 F, identical seeds to exp23), medium 2.83/4 (0.83 F; s0 4/4, the first
full medium run). Some robots now come off (small s1, s5 end free after
346-378 s tilted); medium s3/s5 robot2 still rocks at 13 deg to the end.
No regression, medium +0.17. **Kept** (commit 0bfa437).

### nav4: planner replacement (approved by Joe 27/11 04:03)

Built on exp23 (exp24 touches only the tilt branch and will be folded in if
nav4 is kept). Stage 1 = nav1 (log-odds occupancy, chassis-inflated hard
A*, goals moved out of obstacles) plus make_nav4.py: unseen cells cost 1.5x
seen floor; hard routes re-validated against the current map every 0.5 s
over the next 2.5 m and re-planned the moment they are blocked; Bug2
wall-following off; A* heuristic weight 1.3. Offline (test_nav4.py,
synthetic maps): 1 m door passed, 0.4 m door refused, goal inside a body
moved out, new wall across a route detected, 24 m cluttered map planned in
5 ms, worst case 174 ms. Running seeds 0-5 from 04:25.

nav4 result, seeds 0-5: small 1.67/4 (0.67 F), medium 2.33/4 (1.17 F)
against exp23 2.67 (0.17 F) / 2.67 (0.83 F). Navigation counters improve
(waypoint timeouts per run small 17.5 -> 15.0, medium 14.8 -> 11.8;
regions written off 4.5 -> 3.7, 4.3 -> 2.5) but finds fall, most on small.
Causes seen in the logs (offline tool `_scratch/traj.py`):
- The startup spin learned 69-86 wall cells instead of ~1000: with
  LO_HIT 0.9 < LO_OCC 1.0 a wall needs two hits, and only 90 beams are
  sampled. The first routes ran on an almost empty map (small s1 robot2
  drove straight at the wall between it and v2 and lost 30 s scanning from
  the wrong side; first small reports came at 64-121 s, against 32-44 s).
- Small s1 robot1 left the building on a route through unseen space and
  sat wedged at (12.3,2.5) for 450 s. An envelope mask (nav4b) cannot stop
  this here: the flyover bbox reaches x = 12.85 because the extracted walls
  are extended lines. Not run.
nav4 as built: **not kept**. nav4c = nav4 with LO_HIT 1.2 (one hit makes a
wall, one clear pass removes it) and 180 sampled beams; running from 06:15.

### nav4c (rejected) and a correction

nav4c, small seeds 0-4: 1.40/4 (0.8 F); medium seeds 0-3: 2.75/4 (1.0 F).
Timeouts per run small 17, medium 12.2. Small is worse still; medium is
level with exp24 (2.83).

**Correction to the nav4 entry:** the "sparse startup map" diagnosis was
wrong. The old binary map paints a 3x3 patch per lidar hit, the log-odds map
one cell, so "1003 wall cells" (old) and "87-95" (nav4/nav4c) count the same
~100 hit cells. nav4c's change (LO_HIT 1.2, 180 beams) moved the count from
69-86 to 87-95 and did not help. The startup map was not the problem.

What the logs do show (traj.py, nav4c small s1 robot1 vs exp24 s1 robot1):
the new planner takes a different first route to the north-west estimate
(straight north-east into furniture low enough that only the IR sees it,
30 s lost, then a dead end), reaches the estimate's area later from the
east and never gets the line of sight that the old route gave at t=24 s.
Small's victims sit in furnished rooms where the route chosen decides what
the camera sees; on medium the new planner is neutral to slightly positive.

Next, a 3-seed ablation on the kept stack (exp24) to find which part hurts:
nav4d = log-odds map only (old soft planner, wall-following on);
nav4e = old binary map + hard inflated planner + route re-check, unknown
cost 1.0, wall-following on.

nav4c final, seeds 0-5: small 1.33/4 (0.67 F), medium 2.67/4 (1.33 F).

### Planner ablation (nav4d, nav4e) - planner line closed

On the kept stack (exp24), seeds 0-2, against exp24's seeds 0-2 (small
2, 3, 3 = 2.67; medium 4, 3, 3 = 3.33):

| | small found (F) | medium found (F) | timeouts/run S / M |
|---|---|---|---|
| nav4d: log-odds map only, old soft planner + wall-following | 1.67 (0.67) | 2.00 (1.33) | 19.3 / 17.0 |
| nav4e: old binary map, hard padded planner + route re-check | 1.33 (0.33) | 1.33 (2.00) | 19.3 / 12.0 |

Every map or planner change tried (nav1, nav4, nav4c, nav4d, nav4e) loses
victims on small and does not gain on medium; nav4d shows that even the
map change alone, under the old planner, costs about a victim per run. The
rest of the controller (homing distances, waypoint arrival radii, timeouts,
scan behaviour) has been shaped around the old binary map's behaviour; a
different map changes where robots go and what the camera sees, and the
measured effect is negative. **Planner line closed**; the old planner stays.
No-improvement streak since exp24: 4 (nav4, nav4c, nav4d, nav4e).

Queued: exp24 seeds 6-11 (`exp24x`) to tighten the kept stack's estimate
(n=12 per world) before choosing between remaining local fixes.

### exp24 on seeds 6-11 (exp24x): seeds 0-5 flattered the kept stack

exp24x small 2.17/4 (0.67 F, seeds 6-11); medium 2.50/4 (1.75 F, seeds
6-9, 10-11 still running). Twelve-seed estimate of the kept stack: small
about 2.4/4 (60%), medium about 2.7/4 (67%). Seeds 0-5 read 2.67 / 2.83.

Loss ranking over the kept stack's runs (falses.py, ground truth offline):
- small: v2 found in 11/12 runs, v3 ~10/12, v4 ~6/12, v1 ~3/12 (v1 lies
  1.4 m from v2);
- medium: v4 10/10, v2 ~9/10, v1 ~6/10, v3 ~3/10;
- near misses (report 1.04-1.31 m from the node): medium v3 four times
  (1.15-1.21 m), small five times;
- medium crate phantom: 8 of 10 runs.
Scoring note (supervisor `_check_victim_proximity`, read only): the verdict
uses the robot's TRUE position within 1.0 m of the victim node; the
reported coordinates are not used. Only where the robot parks matters.

exp25 queued: flank a body that extends away from the robot, detected by
the depth spread of its denim (p90 - p10 >= 0.35 m) instead of exp18's
width test. Also logs depth, width and depth spread at every report.

exp24x final (seeds 6-11): small 2.17 (0.67 F), medium 2.50 (1.67 F).
Kept stack over 12 seeds: small 2.42/4 (60%), medium 2.67/4 (67%).

### exp25: flank bodies by denim depth spread (rejected)

Seeds 0-5: small 2.83/4 (0.17 F; s5 4/4), medium 2.33/4 (1.83 F), against
exp24 on the same seeds 2.67 (0.17 F) / 2.83 (0.83 F). Small up, medium
down 0.5: **rejected** under the protocol.
What the new per-report telemetry shows: the flank fired 9 times in 12
runs, 6 of them in medium on the crate cart at (15,-11) robot frame, whose
denim spreads 0.49-0.87 m in depth; flanking it moved the phantom report
from 3.4 m to 4.5 m from victim1, still false. True reports mostly show a
spread of 0.15-0.31 m, but several true small reports show 1.0-1.6 m
(denim of two bodies in one blob, or a body seen along its length), so the
spread does not single out the near-miss cases either. Medium v3's near
miss (1.15 m) recurred in s5.
No-improvement streak since exp24: 5 (nav4, nav4c, nav4d, nav4e, exp25).

### Reality check: kept stack (exp24) at 180 s on the untouched worlds (`v180`)

Seeds 0-2, original worlds and supervisor, MISSION_S 180:
small 1, 3, 3 = 2.33/4 (0 F); medium 3, 2, 2 = 2.33/4 (0.33 F);
large 0, 0, 0 /5 (both robots flip within 4-29 s in every run: the
duplicate-floor defect). At 600 s the same seeds gave small 2.67, medium
3.33: most finds happen before 180 s, so the curriculum costs about half a
victim per world. Overall supervisor score small s0 0.566.

### Where the kept stack loses victims (missed.py, 12 small + 10 medium runs)

Closest odometry approach to each missed victim and the vision verdicts
logged within 1.5-3 m:
- small v1 (missed 9 of 12): robot2 reports v2 from about (4.1,-3.3), 1.4 m
  from v1, then logs 30-200 candidate frames of v1 blocked
  `near-prior-report` / `cooldown`. Seen, not allowed to home.
- small v4 (missed 6 of 12): robots stand 0.7-1.9 m from it for up to
  438 s; every denim frame is rejected `denim-area-small`, visible area
  0.026-0.031 m2 against the 0.032 gate, at floor height.
- medium v1 (missed ~4 of 10): robots within 0.4-1.6 m see no denim at all.
- medium v3 (missed ~7 of 10): robot1 within 1.0-2.2 m; few candidates,
  some blocked by fp-zone / near-prior-report; otherwise near misses.

exp26 queued: lower the denim area gate to 0.022 m2 only within 3 m of an
unreported flyover estimate (small v4).
