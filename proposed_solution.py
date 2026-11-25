"""
UniSC ground-robot controller — IEEE SMCS SAR Competition, Phase 1.

BOTH robots run this one file; identity comes from robot.getName() ->
"robot1" / "robot2", and the two coordinate over radio channel 73.

Mission outline
---------------
1. PREFLIGHT (before the control loop, so it costs no simulation time).
   robot1 analyses the flyover recording for the current world via
   prepare_mission.py; robot2 waits for its outputs. Produces the two graded
   deliverables plus an internal planning grid:
       sim_logs/victim_location_estimates.csv  headerless "x,y" per victim
       sim_logs/map_estimate.png               600x600 @ 0.05 m/px, walls black
       sim_logs/nav_grid.npy                   origin-anchored occupancy grid

2. PLANNING. The victim estimates are split between the robots by brute-force
   partition minimising total A* route cost — computed identically on both
   robots from fixed start positions, so they agree without negotiating.

3. NAVIGATION is LIDAR-FIRST. The routing grid starts EMPTY: the flyover wall
   map is metre-accurate at best, and planning on it drove robots into walls.
   A 3.5 s startup spin paints the surrounding walls, then lidar continuously
   paints and erases cells. A* uses a soft finite wall cost so a route always
   exists; the four IR sensors (mounted at 0.053 m) catch obstacles beneath the
   lidar plane, and an accelerometer tilt guard backs the robot off things it
   is climbing.

4. CONFIRMATION. A victim is reported only after a gated camera chain — denim
   blob fraction, colour ratio, median depth, height above floor and physical
   blob area — holds for REPORT_STREAK_N consecutive frames AND the robot has
   physically closed to within ~0.75 m. Scoring requires the robot's TRUE
   position to be within 1.0 m of the victim marker, so reporting early is
   worthless. Reports go out on channel 43 as the JSON contract the supervisor
   expects; a wrong report is the single most expensive mistake in the rubric
   (it costs roughly 0.065 overall, against 0.027 for missing a victim
   entirely), so every gate here is deliberately conservative.

5. THE MAP DELIVERABLE is upgraded during the mission. Both robots persist
   their lidar-learned grids; robot1 merges them and overwrites map_estimate.png
   late in the mission and immediately before every report (the simulation ends
   the instant the last victim is found). The overwrite is gated on observed
   coverage: below LEARNED_MIN_COVER the flyover map is kept, because a
   sparsely-explored learned map scores worse than the flyover one.

Tuning constants below carry the measurement that set them. Several features
are behind flags defaulting to OFF because controlled A/B runs showed they
cost marks — the reasoning is recorded so they are not re-introduced blindly.
"""

# Ensure the venv's site-packages are on sys.path regardless of which Python
# binary Webots was configured to use.
#
# Venv structure differs by platform:
#   Windows  → .venv/Lib/site-packages          (capital L, no version subdir)
#   macOS/Linux → .venv/lib/pythonX.Y/site-packages
#
# The venv may sit in several places and we control none of them:
#   * beside THIS file — the judges "install your virtual python environment"
#     into controllers/proposed_solution/, which is where our repository's
#     contents are cloned, so .venv/ lands next to this script;
#   * two levels up — our own development tree keeps one at the repo root;
#   * nowhere at all — dependencies already on Webots' Python.
# Searching every candidate (and doing nothing when the imports already work)
# is what makes the controller portable. Getting this wrong is silent and
# fatal: a fresh-clone rehearsal on 30/07 crashed both controllers with
# "No module named 'cv2'" and the mission ran blind, scoring 0.074.
import sys, os as _os
def _add_venv_packages():
    try:                                    # already importable? leave it be
        import cv2, numpy                   # noqa: F401
        return
    except Exception:
        pass
    here = _os.path.dirname(_os.path.abspath(__file__))
    candidates = [
        _os.path.join(here, ".venv"),                             # judges
        _os.path.normpath(_os.path.join(here, "..", "..", ".venv")),  # dev tree
        _os.path.normpath(_os.path.join(here, "..", ".venv")),
        _os.path.join(here, "venv"),
        _os.path.normpath(_os.path.join(here, "..", "..", "venv")),
    ]
    for venv in candidates:
        win_sp = _os.path.join(venv, "Lib", "site-packages")
        if _os.path.isdir(win_sp):
            if win_sp not in sys.path:
                sys.path.insert(0, win_sp)
            return
        lib = _os.path.join(venv, "lib")
        if _os.path.isdir(lib):
            added = False
            for entry in sorted(_os.listdir(lib)):
                sp = _os.path.join(lib, entry, "site-packages")
                if _os.path.isdir(sp):
                    if sp not in sys.path:
                        sys.path.insert(0, sp)
                    added = True
            if added:
                return
_add_venv_packages()
del _add_venv_packages

import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np

from controller import Robot

# ── Pre-mission: flyover video analysis ──────────────────────────────────────

# How long non-robot1 controllers wait for robot1's flyover analysis before
# giving up and analysing themselves. Analysis measures ~2 min per world;
# 8 min covers a slow judging machine with margin.
PREFLIGHT_WAIT_S = 480.0


def run_preflight(world_path=None, robot_name=""):
    """
    Analyse flyover video if outputs are missing or stale.

    world_path: the currently-open .wbt path from Robot.getWorldPath(), so
    find_recording() can match it directly instead of guessing from world
    file timestamps — which broke whenever another Webots window (a
    different world) touched its own file's access time more recently, or
    whenever a world file's *edit* time (not open time) happened to be newest.

    robot_name: both controller processes hit this function at startup.
    Without coordination they analyse the SAME video concurrently — doubling
    wall-clock and racing each other's writes to the shared sim_logs outputs.
    Only robot1 analyses; every other robot polls for robot1's finished
    outputs (Webots does not advance sim time until all controllers step, so
    the wait is free), then falls back to analysing itself if robot1 never
    delivers (e.g. its process died).
    """
    try:
        import prepare_mission as pm
    except Exception as e:
        print(f"[preflight] WARNING – could not import prepare_mission: {e}")
        return

    mp4, imu = pm.find_recording(world_path)
    if mp4 is None:
        print("[preflight] No flyover recordings found – skipping video analysis")
        return
    if pm.outputs_are_fresh(mp4):
        print(f"[preflight] Outputs up-to-date for {os.path.basename(mp4)}")
        return

    if robot_name and robot_name != "robot1":
        print(f"[preflight] {robot_name} waiting for robot1's analysis of "
              f"{os.path.basename(mp4)}…")
        deadline = time.monotonic() + PREFLIGHT_WAIT_S
        while time.monotonic() < deadline:
            time.sleep(2.0)
            if pm.outputs_are_fresh(mp4):
                print(f"[preflight] {robot_name}: outputs ready — proceeding")
                return
        print(f"[preflight] {robot_name}: timed out after {PREFLIGHT_WAIT_S}s "
              f"— analysing independently")

    print(f"[preflight] Analysing {os.path.basename(mp4)}…")
    _analyse_both(pm, mp4, imu)


def _analyse_both(pm, mp4, imu):
    """Run BOTH extraction pipelines, each for what it is measurably better at.

    Two graded deliverables come from `flyover_extraction` (Zeke's pipeline):
    the victim CSV and the wall map. Its victim positions land 0.30-0.61 m from
    the scored markers against our 1.45-5.06 m — see that file's header for the
    measurements — and since the scoring radius is 1.0 m, a waypoint that
    accurate turns "arrive" into "score".

    Our own `prepare_mission` still runs, but only for `nav_grid.npy`: the
    origin-anchored planning grid and the flyover wall bounding box that the
    controller needs for sweep containment and for the learned-map publish
    gate. Its CSV and map are redirected to sim_logs/uni_extract/ so that the
    graded files can only ever come from one place.

    Order matters: ours first (it would otherwise overwrite the deliverables),
    his second, so his outputs are the ones left in sim_logs/.
    """
    side = os.path.join(pm.SIM_LOGS, "uni_extract")
    try:
        os.makedirs(side, exist_ok=True)
        # Redirect ours. SIM_LOGS itself is left alone on purpose: nav_grid.npy
        # is written relative to it and IS wanted in the real directory.
        pm.CSV_OUT   = os.path.join(side, "victim_location_estimates.csv")
        pm.MAP_OUT   = os.path.join(side, "map_estimate.png")
        pm._LAST_REC = os.path.join(side, ".last_recording")
        victims, wall_map = pm.analyse(mp4, imu)
        pm.save(victims, wall_map)
        print("[preflight] our extraction kept for nav_grid.npy only "
              "(CSV/map in sim_logs/uni_extract/)")
    except Exception as e:
        print(f"[preflight] WARNING – our analysis failed ({e}); "
              f"no nav grid, so sweep containment falls back to lidar only")

    try:
        import flyover_extraction as fx
    except Exception as e:
        # Without his pipeline there would be NO graded deliverable, since ours
        # was just diverted — so copy ours back into place rather than ship
        # nothing at all.
        print(f"[preflight] WARNING – could not import flyover_extraction ({e})"
              f" — falling back to our own estimates")
        _promote_side_outputs(pm, side)
        return

    fx.SIM_LOGS = pm.SIM_LOGS
    fx.CSV_OUT  = os.path.join(pm.SIM_LOGS, "victim_location_estimates.csv")
    fx.MAP_OUT  = os.path.join(pm.SIM_LOGS, "map_estimate.png")
    for _attr in ("WAYPOINT_META_OUT", "META_OUT"):
        if hasattr(fx, _attr):
            setattr(fx, _attr, os.path.join(
                pm.SIM_LOGS, os.path.basename(getattr(fx, _attr))))
    try:
        v, m = fx.analyse(mp4, imu)
        fx.save(v, m)
        print("[preflight] graded deliverables written by flyover_extraction "
              "(Zeke's pipeline)")
    except Exception as e:
        print(f"[preflight] WARNING – flyover_extraction failed ({e}) — "
              f"falling back to our own estimates")
        _promote_side_outputs(pm, side)
        return

    # Stamp the cache key for the REAL deliverables. outputs_are_fresh() reads
    # this; without it every run would re-analyse from scratch (~4 min of wall
    # clock, though no sim time — Webots waits for our first step()).
    try:
        with open(os.path.join(pm.SIM_LOGS, ".last_recording"), "w") as f:
            f.write(f"{os.path.abspath(mp4)}\n{pm._code_stamp()}\n")
    except OSError as e:
        print(f"[preflight] note: could not write cache key ({e})")


def _promote_side_outputs(pm, side):
    """Fallback: make OUR diverted CSV/map the graded deliverables again."""
    import shutil
    for name in ("victim_location_estimates.csv", "map_estimate.png",
                 ".last_recording"):
        src = os.path.join(side, name)
        if os.path.exists(src):
            try:
                shutil.copy2(src, os.path.join(pm.SIM_LOGS, name))
            except OSError as e:
                print(f"[preflight] could not restore {name}: {e}")


def load_victim_waypoints():
    """Load pre-computed victim positions from CSV.  Returns [(x, y), …].

    Deliberately built from SIM_LOGS rather than pm.CSV_OUT: the preflight
    redirects pm.CSV_OUT into sim_logs/uni_extract/ (our estimates are kept only
    for nav_grid.npy), and it does so in the analysing process ONLY — so reading
    pm.CSV_OUT would give robot1 our estimates and robot2 the graded ones. The
    graded CSV is always the one in sim_logs/.
    """
    import prepare_mission as pm
    csv_path = os.path.join(pm.SIM_LOGS, "victim_location_estimates.csv")
    if not os.path.exists(csv_path):
        return []
    waypoints = []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                x, y = line.split(",")
                waypoints.append((float(x), float(y)))
            except ValueError:
                pass
    return waypoints


# Robot() itself only opens the IPC connection to Webots (wb_robot_init) — it
# doesn't start the simulation clock, so doing preflight analysis before the
# first step() call is still "outside the sim loop" in the sense that matters
# (no wasted simulation time), while giving us getWorldPath() up front.
_robot = Robot()
run_preflight(_robot.getWorldPath(), _robot.getName())

# ── Webots controller ─────────────────────────────────────────────────────────

# Mission length, seconds. The competition fixes it at 180; every behaviour
# keyed to the mission clock derives from this one constant. SAR_MISSION_S
# exists only for the development time curriculum (dev/tools/run_batch.py
# --mission-s) and is never set in a judged run.
MISSION_S = float(os.environ.get("SAR_MISSION_S", "180"))

WHEEL_RADIUS = 0.043   # metres (Rosbot proto WHEEL_CYLINDER radius 0.043 — was
                       # 0.05, a 16% odometry scale error)
AXLE_LENGTH  = 0.19    # metres
MAX_SPEED    = 14.0    # rad/s commanded cruise (proto maxVelocity is 26; the old
                       # 5.0 capped the robot at 0.25 m/s — too slow to reach
                       # multiple victims inside the 3-minute mission)

# Competition-defined starting positions (origin-marker frame)
START_POS = {"robot1": (-0.375, 0.375), "robot2": (-0.375, 0.0)}

# ── Victim-reporting policy: GATE then report HIGH ──────────────────────────
# The supervisor scores a victim_found report TRUE only when the robot's real
# position is within 1.0 m of the victim marker (see sar_marking_supervisor
# _check_victim_proximity, radius 1.0). Detecting a victim at range is NOT
# enough — we must physically close to <1 m before reporting, or the report
# scores nothing and logs a false positive (which hurts the confidence score,
# the example team's F). So: detect at range, HOME IN, then report once HIGH.
HIGH_CONFIDENCE = 0.9    # fixed value sent once a victim is genuinely confirmed

# ── Victim approach (homing) ─────────────────────────────────────────────────
APPROACH_ENTER_CONF = 0.30          # camera confidence to begin homing
APPROACH_ENTER_M    = 4.0           # only start homing if victim within this range
APPROACH_STOP_M     = 0.75          # depth at which we report. MEASURED 21:51:
                                    # camera depth to a lying victim's denim
                                    # bottoms out at ~0.62-0.66 m (chassis +
                                    # body geometry) — the 0.60 experiment left
                                    # both robots parked AT victims, conf 0.95,
                                    # vetoing every report. The streak-phase
                                    # creep below squeezes out the rest.
APPROACH_SPEED      = MAX_SPEED * 0.45
HOME_TURN_GAIN      = 2.5           # steering gain toward the victim blob
APPROACH_TIMEOUT_S  = 12.0          # give up homing and resume the route after this
APPROACH_LOST_S     = 1.5           # victim out of view this long → abort homing
                             # body seen end-on has its scored centroid ~0.85 m
                             # beyond the visible surface, which reads as a
                             # FALSE report (large_world 30/07, -0.06 overall)
REPORT_STREAK_N     = 8             # consecutive gate-passing frames before reporting
                                    # (4 = 0.13 s — a marginal decoy passed that long
                                    # on large 29/07 and cost a 0.9-conf false report)

# ── Vision gate hardening (window / sky / speckle countermeasures) ───────────
# The denim mask on its own is NOT a victim detector. Sky seen through a
# window, blue-tinted glass and scattered blue speckle all satisfy it, and the
# mission logs show exactly one such report firing at t≈6-15 s in every run,
# ~2.8 m from the nearest real victim. Three countermeasures, all geometric
# rather than world-specific (the judged worlds are unseen):
#   1. LARGEST CONNECTED denim component, not the raw pixel count — a body is
#      contiguous, speckle scattered across the frame is not.
#   2. MEDIAN depth over that component, not np.min over the whole mask. One
#      stray blue pixel on a near surface used to collapse min_depth below the
#      0.80 m report gate while the actual denim was metres away.
#   3. Reconstructed HEIGHT ABOVE FLOOR must be consistent with a person
#      standing on it. Victim denim spans roughly 0.2–1.1 m and reaches down
#      toward the floor; window glass sits at ~0.9–2.0 m and never does.
DENIM_MIN_COMPONENT_FRAC = 0.0025  # largest denim blob, as a fraction of frame
CAM_HEIGHT_M             = 0.19    # Astra optical centre above the floor (proto)
VICTIM_DENIM_MAX_H_M     = 1.40    # median denim height above this → not a person
VICTIM_DENIM_LOW_H_M     = 0.90    # some denim must sit below this (legs on floor)
#   4. PHYSICAL AREA of the denim blob (pixels × depth² / focal²) must be
#      person-sized. The 17:32 run's decoy at mat (0.79, 3.24) — an ~8×8 cm
#      blue object at floor level — passed every height/blob-fraction test
#      because those scale with distance; its physical area (~0.007 m²) cannot
#      reach a pair of trousers (>0.05 m² even partly occluded). World-agnostic.
VICTIM_DENIM_MIN_AREA_M2 = 0.032   # smallest credible visible denim area.
                                   # Measured: decoy 0.028-0.0295 m2 (18:27 and
                                   # 21:51 runs); UNOCCLUDED victims at report
                                   # range >=0.067; the bed-occluded victim3
                                   # reads 0.037-0.038 (22:03 run — the 0.040
                                   # gate rejected a real victim all mission).
                                   # 0.032 splits the measured gap; the 4-frame
                                   # streak guards the thin margin.

# ── Marker-offset countermeasure (PLAN §3.5) ────────────────────────────────
# Victim markers sit up to ±1.3 m from the visible body. 22:03 run: parked
# 0.78 m from the denim, reported, scored FALSE at 1.37 m from the marker.
# The CSV estimates are better marker proxies (~0.8-1.0 m). So after locking
# the denim, drive toward the nearest unreported CSV estimate and report from
# there instead of from the body.
REPOSITION_MAX_M    = 2.2   # estimate must be this close to trigger reposition
REPOSITION_MIN_M    = 0.9   # already this close to the estimate → just report
REPOSITION_STOP_M   = 0.45  # report once within this distance of the estimate
REPOSITION_TIMEOUT_S = 6.0  # give up and report from wherever we got to

# ── Vision telemetry ────────────────────────────────────────────────────────
# Diagnostic only: prints why the detection gate did or did not open. Costs one
# extra CV pass every DEBUG_VISION_EVERY_S. SET False BEFORE SUBMISSION.
DEBUG_VISION         = os.environ.get("SAR_DEBUG_VISION") == "1"
                               # Off unless the dev harness sets the env var
                               # (run_batch.py --debug-vision). True for development runs:
                               # prints per-tick vision telemetry (costs an
                               # extra CV pass every DEBUG_VISION_EVERY_S).
DEBUG_VISION_EVERY_S = 0.5

# ── Route planning: world-agnostic ──────────────────────────────────────────
# No world-specific coordinates here — small/medium/large/basic_example_sar
# all have different room layouts, so anything hardcoded to one world's
# doorways/victim positions silently fails on the others.
SWEEP_HALF_EXTENT_M = 30.0   # matches prepare_mission.py's GRID_HALF_M (60 m world
                              # grid) — medium_world's walls alone reach ~(+20,-27)
SWEEP_STEP_M        = 3.0    # spacing between sweep rows/columns
WAYPOINT_TIMEOUT_S  = 40.0   # hard cap on any single waypoint
WAYPOINT_STALL_S    = 20.0   # abandon once the distance stops improving
WAYPOINT_STALL_MIN_DIST_M = 2.5  # ...but never while already close to it
# Frontier exploration: measured 30/07 on all three worlds (Webots is
# deterministic, so one run per world is conclusive). It cost marks everywhere
# — mean overall 0.480 with it vs 0.498 without — because the extra driving hurt
# Efficiency without finding additional victims. Kept behind this flag: it is
# the right answer for a longer mission, but not inside 180 s.
# ─────────────────────────────────────────────────────────────────────────────
#  TUNING FLAGS — every default below was set by a controlled A/B run scored by
#  the competition's own marking server (one constant changed at a time,
#  hermetic runs, see docs/MEETING_BRIEF.md for the full table). Flags that are
#  False are features that MEASURED WORSE, kept so the finding is not lost.
# ─────────────────────────────────────────────────────────────────────────────
EXPLORE_FRONTIERS   = False
# Progress-based waypoint abandonment: also measured worse (medium 0.437 with
# vs 0.475 without). The flat WAYPOINT_TIMEOUT_S cap remains in force.
STALL_ABANDON       = False
ASSIST_ENABLED      = True   # PLAN §3.6 partner assist (A/B control)
ASSIST_SKIP_FAILED  = True   # skip targets either robot abandoned (A/B control)
# Blocking wall cells outright (two-tier planning) measured slightly WORSE than
# the soft cost: mean 0.468 vs 0.472, and medium lost a victim (2/4 -> 1/4).
# Our painted map is noisy — 3x3 patches at every lidar endpoint — so hard
# blocking also seals narrow doorways the robot could otherwise squeeze
# through. The finite cost acts as a safety valve. Kept as a flag.
HARD_WALL_FIRST     = False
# A robot that cannot reach one target usually cannot reach its NEIGHBOURS
# either — they are behind the same obstruction. Measured on medium 30/07:
# robot1 reached (10.3,-15) at t=60 and then spent 110 s oscillating within a
# 3 m box while all four of its remaining waypoints sat 5-6 m away in the same
# blocked region; it scored nothing, which also halved Coordination (that
# component is binary on both robots scoring). After REGION_FAIL_N failures in
# one area we blacklist a REGION_FAIL_R radius around it and skip targets
# inside, so the robot spends its remaining seconds somewhere it can reach.
# Measured 30/07: writing off regions COST medium a victim and produced a
# false report (0.479 -> 0.407) — it pulled a robot away from an area it would
# eventually have reached. Off by default; the sink it was meant to fix is real
# but this cure is worse than the disease.
REGION_BLACKLIST    = False
# WALL FOLLOWING (Bug2-style trap escape). Measured signature on small_world
# 30/07: both robots closed to 1.6-2.5 m of their victim waypoints and then
# oscillated there for 120 s, finding nothing. Cause: a soft-cost A* path can
# cut straight through a wall, so the robot drives at the wall, local avoidance
# sidesteps, the planner re-proposes the same through-wall route, and the cycle
# repeats. Detect "close but no progress" and follow the obstacle surface
# instead — that is what carries a robot along a wall to its doorway.
# Enabled with CONSERVATIVE timings. The aggressive version (12 s trigger, 12 s
# follow) cost small_world 0.106: not by failing to escape traps but by hugging
# walls instead of covering rooms, which dropped lidar coverage below
# LEARNED_MIN_COVER so the good learned map never published (extraction 0.663 ->
# 0.318). At 20 s / 6 s it fires 4x on small and is score-neutral there (0.547,
# extraction 0.683) and on medium (0.479) — it escapes traps without spending
# the coverage. Kept ON deliberately: trapping is a real, observed failure mode
# and the judging worlds are unseen, so a benign escape hatch is worth having.
#
# 30/07 evening, from top-down screenshots of the worlds: the reason a 6 s follow
# could not fix this is GEOMETRY. Both robots start on a mat OUTSIDE the
# building (small_world: on the paving south of the house; medium_world: in the
# truck bay, a separate hall). small_world's robot2 oscillated at mat y = -6.0
# with its two victims at world y = -10.7 and -11.4, i.e. it was pressed against
# the OUTSIDE of the south wall, 2-3 m from victims on the other side of it, and
# the front door is ~2-3 m further WEST along that wall. At 0.33 m/s a 6 s
# follow covers 2 m — just short of the door, every time.
# So the follow now terminates on GEOMETRY, not on a stopwatch (true Bug2): keep
# following the surface until the robot is measurably closer to the goal AND the
# lidar shows an open lane in the goal's direction — that is what "we have
# reached the doorway" looks like from the robot's point of view. The timer
# survives only as a safety cap. This is what makes it different from the
# rejected aggressive version: the exit test fires as soon as the door is found,
# so the robot returns to room coverage instead of hugging wall indefinitely.
WALL_FOLLOW          = True
WF_TRIGGER_DIST_M    = 5.0   # only bother when the target is already near
WF_NO_PROGRESS_S     = 20.0  # no improvement for this long -> trapped
WF_PROGRESS_EPS_M    = 0.5   # improvement that counts as progress
WF_MAX_S             = 12.0  # SAFETY cap only; the leave test normally ends it
WF_KEEP_M            = 0.55  # distance to hold from the followed surface
WF_SPEED_F           = 0.55  # fraction of MAX_SPEED while following
WF_LEAVE_GAIN_M      = 0.6   # must be this much closer than the hit point
WF_LEAVE_CLEAR_M     = 1.5   # open lane toward the goal that counts as a door
WF_LEAVE_CONE_DEG    = 70.0  # goal must be roughly ahead to test that lane
WF_HANDBACK_M        = 1.2   # this close: hand straight back to the planner
WF_LOOP_M            = 1.0   # back at the hit point => surface circumnavigated
# MEASURED AND REJECTED (30/07): crediting the waypoint deadline with the time
# spent following (so a follow could not consume the approach budget) cost
# small_world 0.551 -> 0.208 in BOTH runs — 0 victims and a false report. The
# waypoint deadline is not overhead, it is what pushes a robot off a hopeless
# target and onto the sweep leg, which is where small_world's only find happens.
# Isolated by A/B: the leave test WITHOUT the credit scores 0.549, i.e. neutral.
# Do not reintroduce a deadline extension without re-measuring both worlds.
REGION_FAIL_N       = 2      # failures in one area before it is written off
REGION_FAIL_R       = 4.0    # metres
# Geofence slack around (flyover bbox ∪ origin mat). Diagnosed 31/07 with
# DEBUG_VISION telemetry: at 3.5 m the fence NEVER fired — robot2 spent 240 of
# ~700 ticks more than 1 m outside the wall bbox (deepest 3.19 m) while its
# targets were interior victims, i.e. orbiting the building the cheap way
# around instead of entering it. Homing was exonerated (conf 0.00 on every
# outside tick) and so were sweep waypoints (the held targets were video
# waypoints). Keep the slack above the extraction's positional error (~1.4 m
# bbox offset on small) but small enough that the escort-back actually engages.
GEOFENCE_SLACK_M    = 3.5    # metres beyond the bbox before steering back
# Partner failures are NOT our failures: the other robot may be blocked from a
# side we can approach from (on medium, robot2 reached the very area robot1
# could not). Skip only targets WE have failed.
ASSIST_SKIP_OWN_ONLY = True   # measured neutral; principled choice
# Victims can be CLOSE together: small_world's victim1 and victim2 are 1.47 m
# apart, so any radius that treats "near a previous report" as "already dealt
# with" must be tighter than that, or reporting one victim cancels the search
# for its neighbour.
# ARRIVAL RADIUS. A victim waypoint is only an ESTIMATE — measured error is
# 1.45 m on small, 3.05 m on large, 5.11 m on medium — so the exact coordinate
# is routinely inside a wall or a sofa and can never be reached. Demanding
# 0.55 m of a point that does not exist is what produced the worst failure mode
# we have: robots sat 1.6-2.5 m from their targets oscillating for 120 s, well
# inside camera range of the victim, refusing to stop and look. Arriving within
# ARRIVE_VIDEO_M and SCANNING from there turns an impossible drive into a look
# around, and homing takes over the moment the camera sees denim. Sweep points
# are real free-space coordinates, so they keep the tight radius.
ARRIVE_VIDEO_M      = 1.6    # victim estimates: close enough to look around
# ...and if we cannot even reach that, LOOK ANYWAY. The robots' worst failure
# mode is oscillating 2-2.5 m short of a victim estimate for 120 s, which is
# well inside camera range. Once we are within SCAN_NEAR_M of a victim estimate
# and have stopped making progress for SCAN_STUCK_S, stop driving and scan on
# the spot: if the victim is visible, homing takes over and closes to under
# 0.75 m before reporting, so scanning early costs nothing and reporting still
# requires real proximity.
SCAN_NEAR_M         = 3.0
SCAN_STUCK_S        = 8.0
ARRIVE_SWEEP_M      = 0.55   # sweep/exploration points: actual positions
WP_RETIRE_M         = 1.2    # retire a victim waypoint this close to a report
# Homing suppression stays WIDE, unlike waypoint retirement. Measured 30/07 on
# large_world: at 1.0 m the second robot re-homed on an already-reported victim
# and fired a FALSE report — confidence 0.99 -> 0.511, costing 0.064 overall.
# Retiring a waypoint merely redirects a robot; homing leads to a REPORT, and a
# wrong report is the most expensive mistake available in this rubric.
HOME_BLOCK_R_M      = 1.5
FRONTIER_REPLAN_S   = 4.0    # how often exploration re-picks a frontier
FRONTIER_MAX_TRIES  = 6      # A* probes per re-pick (keeps the tick cheap)

# On arrival, rotate in place scanning for a victim rather than checking only
# the heading we happened to arrive facing. A hardcoded route can stage the
# approach angle so the victim is already ahead of the camera; a generic
# route (video waypoints / sweep grid) can't assume that.
SCAN_DURATION_S  = 6.0
SCAN_TURN_SPEED  = 2.0   # wheel rad/s (pure rotation)

# ── Obstacle handling ────────────────────────────────────────────────────────
DANGER_FRONT_M   = 0.28   # closer than this dead ahead → reverse.

# ── Low-obstacle (IR) + tilt guards ──────────────────────────────────────────
# 29/07 three-world runs: large_world = both robots FLIPPED on rubble within
# 15 s (0/5, official 0.046); medium_world = robot wedged on stairs. Both
# obstacle classes sit BELOW the lidar plane (~0.13 m). The Rosbot's four IR
# range sensors are mounted at 0.053 m and see them; the accelerometer sees
# the climb before the flip.
IR_DANGER_M        = 0.35   # front IR closer than this → treat as wall ahead
IR_CLEAR_V         = 1.9    # IR lookup maxes at 2 m; ≥ this means "nothing"
TILT_ABORT_DEG     = 11.0   # climbing something → back off before it flips us
TILT_ESCAPE_S      = 1.2    # reverse duration after a tilt trigger
# An accelerometer reads gravity PLUS linear acceleration: speed steps look
# like tilt (0.8·MAX in one tick ≈ tens of degrees of apparent lean). The raw
# reading triggered a reverse right as the startup spin handed over to full
# cruise, whose own jerk re-triggered it — the bang-bang rocking flipped both
# robots on the mat (29/07). Low-pass + sustained-exceedance separates real
# climbing (persistent) from command jerk (spikes).
TILT_EMA_TAU_S     = 0.4    # low-pass time constant for the tilt estimate
TILT_SUSTAIN_S     = 0.35   # tilt must stay high this long before we react
# Wheel-command slew limiting: commanded velocity steps of ±MAX in one tick
# torque-spike the chassis (wheelies) and hammer an already-fragile physics
# solver (large_world spams "physics step could not be computed" whenever the
# robots move — solver failures can inject the very impulses that flip them).
SLEW_RAMP_S        = 0.35   # 0 → MAX_SPEED command ramp time
# Tilt-recovery: 25–80° means on our side / high-centred on rubble — worth
# trying to rock back onto the wheels. Beyond 80° we are inverted (wheels-up:
# nothing wheels can do); freeze so odometry and the map stay unpoisoned.
TILT_RECOVER_LO    = 25.0
TILT_RECOVER_HI    = 80.0
RECOVER_PULSE_S    = 0.6    # alternate fwd/rev pulse length
RECOVER_MAX_S      = 8.0    # wiggle burst length per attempt
RECOVER_REST_S     = 12.0   # rest between attempts — retry forever (a beached
                            # robot has nothing better to do; 29/07 large run:
                            # permanent give-up stranded both robots at 25°)
LIDAR_TILT_MAX_DEG = 6.0    # don't paint/erase the nav grid from a tilted
                            # lidar (large's learned map scored 0.003 — a
                            # flipped robot painted the floor as walls)
                          # MEASURED A/B (22:20 vs 22:36 runs): raising this to
                          # 0.34 with a 0.38 sidestep threshold and speed
                          # scaling pinned robot1 in a doorway for 100+ s
                          # (doorframes always read <0.38 ahead) — 3/4 found
                          # collapsed to 1/4. Doorways demand assertiveness;
                          # do not re-tighten these without a full A/B.
CLEAR_FRONT_M    = 0.40   # narrow-cone clearance needed to drive toward the goal
STUCK_WINDOW_S   = 4.0    # if we move < STUCK_DIST_M within this window → escape
STUCK_DIST_M     = 0.30
ESCAPE_S         = 2.5    # duration of an escape maneuver (reverse + turn)

# ── Global route planning (soft-cost A* on the flyover wall map) ─────────────
# The flyover map (nav_grid.npy, origin-anchored) is too rough to steer on
# directly, but good enough to route THROUGH the right doorways. We plan an A*
# path to each victim and follow it as sub-waypoints; the lidar reactive layer
# handles the imperfect/furniture bits locally.
#
# PLAN.md Part 1, measured on 28/07: ERODING the grid to "reopen doorways"
# deleted the walls themselves (2.1k of 360k cells survived — this run's
# "drivable 357883/360000") and every route degenerated to a straight line
# into a foyer wall. NEVER erode. Instead walls are SOFT: crossing a wall cell
# costs high-but-finite, so real walls repel routes toward real doorways while
# a phantom wall sealing a doorway gets crossed at its thinnest point instead
# of making a victim "unreachable". Clearance comes from a cheap near-wall
# cost ring (inflation only). The lidar learner (below) then converges the
# grid toward truth from both sides while driving.
NAV_RES           = 0.10   # planning-grid resolution (m/px)
STARTUP_SPIN_S    = 3.5    # 360° lidar spin before the first route commits:
                           # paints the foyer's REAL walls (5 m lidar horizon)
                           # into the grid so the initial route exits through a
                           # real doorway instead of darting at a hidden wall
NAV_WALL_COST     = 30.0   # per-cell multiplier to CROSS a wall cell (finite!)
NAV_NEAR_COST     = 3.0    # cost ring within NAV_INFLATE_M of a wall (clearance)
NAV_INFLATE_M     = 0.20   # clearance ring radius around walls
NAV_SUBGOAL_M     = 0.55   # distance at which a sub-waypoint counts as reached
NAV_REPLAN_S      = 5.0    # re-plan the route at least this often
NAV_ASTAR_BUDGET  = 90000  # max cells A* expands before giving up (bounds stall)

# ── Two-way lidar map learning (PLAN §3.4) ──────────────────────────────────
# Lidar endpoints paint walls INTO the grid; the free space each ray passes
# through erases phantom walls OUT of it. The flyover map self-corrects from
# both sides while driving; painting triggers a re-plan so the robot routes
# around a wall it just met instead of grinding into it.
LIDAR_MAP_EVERY_S = 0.3    # how often the scan is folded into the grid
LIDAR_PAINT_MAX_M = 5.0    # trust hits nearer than this as real walls
LIDAR_ERASE_MAX_M = 3.0    # erase phantom walls along the ray up to this
LIDAR_MAP_BEAMS   = 90     # beams sampled per update (of the full scan)

# ── End-of-mission learned-map deliverable ──────────────────────────────────
# The marking supervisor reads map_estimate.png in save_logs() AT MISSION END
# (verified), so the deliverable can be rewritten late in the mission from the
# lidar-learned grid: painted walls are centimetre-accurate measurements, and
# flyover walls that a robot's lidar actually observed-and-confirmed survive,
# while unobserved flyover guesswork (measured IoU 0.057 on small_world) is
# dropped. Both robots dump their learned grids; robot1 merges and writes.
LEARNED_SAVE_EVERY_S  = 5.0    # cadence of learned-grid dumps to sim_logs
LEARNED_WRITE_TIMES   = (MISSION_S - 8.0, MISSION_S - 2.0)   # robot1 merges/writes
                                         # (172 s and 178 s in the 180 s mission)
# DISABLED (1.01 = unreachable) NOW THAT WE SHIP A BETTER FLYOVER MAP.
# This gate let the learned map replace the flyover map once the robots had seen
# 30% of the envelope. It earned its place while the flyover map was OURS and
# poor (small_world ~0.46). With Zeke's extraction supplying the map, the premise
# is gone, and the learned map turns out to be a lottery — same build, same
# world, same coverage, wildly different marks:
#
#   small_world learned map, official scores:  0.906  0.144  0.144
#                                              (cov 45%, 47%, 47%)
#   small_world flyover map (Zeke's):          ~0.56, every run
#
# Cause is understood (see docs/logged_attempts.md): the ground truth contains
# WALLS ONLY and lidar cannot tell a wall from a sofa, so what the robots add is
# part measurement and part furniture, in a proportion nobody controls. We are
# judged on ONE run per world, so a stable 0.56 beats a coin flip between 0.14
# and 0.91. medium and large are unaffected — their coverage never reached the
# gate anyway, which is why their maps already came from the flyover (0.308 and
# 0.799 against our 0.457 and 0.112).
LEARNED_MIN_COVER     = 1.01   # fraction of the building envelope the robots
                               # must have observed before the learned map is
                               # allowed to replace the flyover map
# DISABLED (1.01 = unreachable). The idea: when the lidar contradicts most of
# the flyover wall it observed, the flyover map is wrong and ours is better even
# when sparse. The signal itself works beautifully — large_world reports 82-95%
# disagreement, medium 94%, small 65% — but publishing on it MEASURED UNSAFE: a
# learned map built from ~11% coverage scored 0.068 in one large run and 0.314
# in another (against the flyover map's 0.112). At low coverage the learned map
# is a coin flip, so coverage remains the gate. The metric is still logged
# because it is the clearest evidence we have that large's flyover extraction is
# badly misaligned.
LEARNED_DISAGREE_FRAC = 1.01
LEARNED_MIN_COVER_FLOOR = 0.08 # ...but never publish from almost no observation
LEARNED_MIN_WALL_PX   = 1500   # deliverable floor (0.05 m/px px) — below this
                               # keep the flyover map rather than a near-empty one


def write_learned_map(sim_logs, prior, grids, bbox_rc=None, flyover=None):
    """
    Merge the robots' learned wall grids into the map_estimate.png deliverable.

    prior  – pristine flyover wall grid (uint8, NxN @ NAV_RES), or None.
    grids  – [(wall, seen), ...] per robot, same shape as prior.

    Deliverable = lidar-painted walls (measurements) + flyover walls that some
    robot's lidar observed and did not erase. Unobserved flyover guesswork is
    dropped. Output follows the marking contract: 600x600 @ 0.05 m/px, pure
    black walls on pure white, window centred on the wall bounding box.
    Returns True if written, False if below the safety floor (flyover map kept).
    """
    base = grids[0][0]
    if prior is None:
        prior = np.zeros_like(base)
    prior_b = prior > 0
    painted = np.zeros(base.shape, bool)
    erased  = np.zeros(base.shape, bool)
    seen    = np.zeros(base.shape, bool)
    for wall_g, seen_g in grids:
        painted |= (wall_g > 0) & ~prior_b
        erased  |= prior_b & (wall_g == 0)
        seen    |= seen_g > 0
    keep_prior = prior_b & seen & ~erased
    merged     = (painted | keep_prior).astype(np.uint8)

    # Clip to the building envelope: lidar paint gathered while wandering
    # OUTSIDE (exterior fences, garden clutter) drags the bbox-centred window
    # away from the building (22:20 run: -2.4 m window shift, IoU halved).
    if bbox_rc is not None:
        r0, r1, c0, c1 = bbox_rc
        m = np.zeros_like(merged)
        m[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = 1
        merged &= m

        # COVERAGE GATE. The learned map only beats the flyover map where the
        # robots actually drove. Measured 29/07: small world (robots roam most
        # of a compact building) learned map scored 0.796–0.889 vs flyover
        # ~0.46; medium (a big building, two rooms visited) scored 0.141 vs
        # the flyover map's 0.457 — overwriting LOST marks. So only publish
        # the learned map once enough of the envelope has been observed.
        box = np.zeros(seen.shape, bool)
        box[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = True
        n_box = int(box.sum())
        if n_box > 0:
            cover = float((seen & box).sum()) / n_box
            # DISAGREEMENT OVERRIDE. Coverage alone is the wrong test when the
            # flyover map is itself wrong: on large_world its walls are ~5 m
            # misaligned and it scores 0.112, while the robots' sparse learned
            # map — only 12% coverage — scores 0.314. The robots know something
            # about this: driving through a claimed wall ERASES it, so the
            # fraction of observed flyover wall that the lidar contradicted is a
            # ground-truth-free measure of how wrong the flyover map is. When
            # most of what we looked at turned out not to be there, publish what
            # we measured instead of what we were told.
            fw = flyover if flyover is not None else prior_b
            if fw.shape != seen.shape:
                fw = prior_b
            obs_prior = int((fw & seen & box).sum())
            contradicted = int((fw & seen & box & ~(merged > 0)).sum())
            disagree = (contradicted / obs_prior) if obs_prior else 0.0
            print(f"[learned-map] coverage {cover:.0%}, flyover disagreement "
                  f"{disagree:.0%} ({obs_prior} observed prior cells)")
            if cover < LEARNED_MIN_COVER and not (
                    disagree >= LEARNED_DISAGREE_FRAC
                    and cover >= LEARNED_MIN_COVER_FLOOR):
                print(f"[learned-map] coverage {cover:.0%} < "
                      f"{LEARNED_MIN_COVER:.0%} — keeping flyover map")
                return False

    # NAV_RES (0.1 m/px) → deliverable scale (0.05 m/px): ×2 upsample.
    hi = np.kron(merged, np.ones((2, 2), np.uint8))
    if int(hi.sum()) < LEARNED_MIN_WALL_PX:
        return False

    ys, xs = np.nonzero(hi)
    G   = hi.shape[0]
    OUT = 600
    cx  = int(round((xs.min() + xs.max()) / 2.0))
    cy  = int(round((ys.min() + ys.max()) / 2.0))
    x0  = min(max(cx - OUT // 2, 0), max(G - OUT, 0))
    y0  = min(max(cy - OUT // 2, 0), max(G - OUT, 0))
    crop = hi[y0:y0 + OUT, x0:x0 + OUT]
    out  = np.full((OUT, OUT), 255, np.uint8)
    out[crop[:OUT, :OUT] > 0] = 0
    cv2.imwrite(os.path.join(sim_logs, "map_estimate.png"),
                cv2.cvtColor(out, cv2.COLOR_GRAY2RGB))
    return True


class SARRobot:
    def __init__(self, robot):
        self.robot    = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_id = self.robot.getName()

        self._init_devices()

        # Odometry — start at competition-defined position
        sx, sy     = START_POS.get(self.robot_id, (-0.375, 0.0))
        self.x     = sx
        self.y     = sy
        self.theta = 0.0

        self._prev_fl = self._prev_fr = None
        self._cam_streak = 0
        self._wp_start_time = None    # simulation time when current waypoint was targeted
        self._wp_best_dist  = float('inf')   # closest approach to it so far
        self._wp_best_t     = 0.0            # when that closest approach happened
        self._avoid_ticks   = 0       # avoidance hysteresis: ignore waypoint heading for N steps
        self._scanning         = False
        self._scan_ticks_left  = 0
        self._scan_duration_ticks = int(SCAN_DURATION_S * 1000 / self.timestep)

        # Victim homing state
        self._homing          = False
        self._home_start_time = None
        self._home_lost_ticks = 0
        self._home_cooldown_until = 0.0
        self._report_streak   = 0
        self._home_best_depth = float('inf')
        self._home_best_t     = 0.0
        self._video_estimates = []
        self._repositioning   = None
        self._repo_deadline   = 0.0
        self._repo_prev       = None
        self._repo_stall_t    = 0.0

        # Detection-gate diagnostics (see _vision_telemetry)
        self._last_vision = {"why": "no-sample"}
        self._home_block  = "-"
        self._dbg_next_t  = 0.0

        # Stuck-escape state
        self._escape_ticks   = 0
        self._last_prog_pos  = None
        self._last_prog_time = None

        # Global route-planning state
        self._nav_wall       = None   # uint8 grid, 1 = wall (soft-cost, live-edited)
        self._nav_cost       = None   # derived traversal-cost grid (lazy)
        self._cost_dirty     = True
        self._route_cost_cache = {}
        self._lidar_map_next_t = 0.0
        self._wp_retries       = {}
        self._assist_next_t    = 0.0     # PLAN §3.6 assist: next re-evaluation
        self._partner_target   = None    # partner's current video waypoint
        self._geofence_active  = False   # currently steering back inside
        self._frontier_next_t  = 0.0     # next frontier re-evaluation
        self._frontier_failed  = []      # unreachable frontiers (don't retry)
        self._explore_count    = 0
        self._failed_wps       = []      # waypoints we abandoned (ours)
        self._fail_regions     = []      # (x, y, failure_count) clusters
        self._wf_until         = 0.0     # wall-following safety deadline
        self._wf_side          = 'right' # which side the surface is on
        self._wf_count         = 0
        self._wf_hit           = None    # (x, y) where the surface was met
        self._wf_hit_dist      = 0.0     # goal distance at the hit point
        self._wf_started       = 0.0     # sim time the follow began
        self._partner_failed   = []      # waypoints the partner abandoned
        self._nav_prior        = None
        self._nav_seen         = None
        self._spin_done        = False
        self._flyover_bbox     = None
        self._sim_logs         = None
        self._learned_next_t   = 0.0
        self._learned_written  = set()
        self._nav_N          = 0
        self._path           = None   # list of (x, y) sub-waypoints toward goal
        self._path_idx       = 0
        self._planned_goal   = None
        self._last_plan_time = -1e9
        self._load_nav_grid()

        # No staggered start — every second of the 180 s mission counts; the
        # A* routes + lidar avoidance handle the shared start corridor.
        self._startup_ticks = 0

        # Partner robot's last known position (from channel-73 position broadcasts)
        self._partner_x = None
        self._partner_y = None

        # Confirmed victim positions (ours + partner's) — used for deduplication
        self._reported_positions = []

        # Plan and order this robot's waypoints once at startup
        all_waypoints      = load_victim_waypoints()
        self._n_video_wps  = 0    # populated by _plan_route
        self._my_waypoints = self._plan_route(all_waypoints)
        self.target_idx    = 0

        n_sweep   = len(self._my_waypoints) - self._n_video_wps
        vid_wps   = self._my_waypoints[:self._n_video_wps]
        vid_str   = ", ".join(f"({x:.1f},{y:.1f})" for x, y in vid_wps) or "none"
        print(f"[{self.robot_id}] Ready  –  {self._n_video_wps} video waypoints + "
              f"{n_sweep} sweep waypoints")
        print(f"[{self.robot_id}] Video waypoints assigned: {vid_str}")

    # ── Device initialisation ─────────────────────────────────────────────

    def _init_devices(self):
        # Motors
        self._motors = {}
        for name, key in [("fl_wheel_joint", "fl"), ("fr_wheel_joint", "fr"),
                           ("rl_wheel_joint", "rl"), ("rr_wheel_joint", "rr")]:
            m = self.robot.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            self._motors[key] = m

        # Wheel encoders
        self._enc = {}
        for name, key in [("front left wheel motor sensor",  "fl"),
                           ("front right wheel motor sensor", "fr"),
                           ("rear left wheel motor sensor",   "rl"),
                           ("rear right wheel motor sensor",  "rr")]:
            s = self.robot.getDevice(name)
            s.enable(self.timestep)
            self._enc[key] = s

        # Compass (heading)
        try:
            self._compass = self.robot.getDevice("imu compass")
            self._compass.enable(self.timestep)
        except Exception:
            self._compass = None

        # Accelerometer (tilt guard) + low-mounted IR range sensors
        try:
            self._accel = self.robot.getDevice("imu accelerometer")
            self._accel.enable(self.timestep)
        except Exception:
            self._accel = None
        self._ir = {}
        for nm in ("fl_range", "fr_range", "rl_range", "rr_range"):
            try:
                d = self.robot.getDevice(nm)
                d.enable(self.timestep)
                self._ir[nm] = d
            except Exception:
                pass
        self._tilt_deg_now      = 0.0    # low-passed tilt estimate
        self._tilt_hi_since     = None   # when the estimate first exceeded ABORT
        self._tilt_escape_until = 0.0
        self._recover_start     = None   # tilt-recovery wiggle start time
        self._cmd_l = self._cmd_r = 0.0  # last slew-limited wheel commands

        # Lidar (obstacle detection)
        try:
            self._lidar = self.robot.getDevice("laser")
            self._lidar.enable(self.timestep)
        except Exception:
            self._lidar = None

        # Forward RGB camera (victim colour detection)
        try:
            self._camera = self.robot.getDevice("camera rgb")
            self._camera.enable(self.timestep)
        except Exception:
            self._camera = None

        # Depth camera (victim range filtering)
        try:
            self._depth_camera = self.robot.getDevice("camera depth")
            self._depth_camera.enable(self.timestep)
        except Exception:
            self._depth_camera = None

        # Supervisor emitter — channel 43, victim-found messages only
        try:
            self._sup_emitter = self.robot.getDevice("supervisor emitter")
        except Exception:
            self._sup_emitter = None

        # Robot-to-robot comms (channel 73)
        try:
            self._squad_tx = self.robot.getDevice("robot to robot emitter")
            self._squad_rx = self.robot.getDevice("robot to robot receiver")
            self._squad_rx.enable(self.timestep)
        except Exception:
            self._squad_tx = self._squad_rx = None

    # ── Route planning ────────────────────────────────────────────────────

    def _plan_route(self, all_waypoints):
        """
        Primary targets are the video-derived victim waypoints (distance-
        ranked from this robot's start, split so both robots get a disjoint,
        roughly even share). Once those are exhausted, fall back to a generic
        sweep grid covering a world-agnostic square — no hardcoded knowledge
        of any one world's room layout. Both legs are nearest-neighbour
        ordered and chained continuously (sweep continues from wherever the
        video waypoints end) so no single leg jumps across the whole map.
        """
        sx, sy = START_POS.get(self.robot_id, (-0.375, 0.0))
        self._video_estimates = list(all_waypoints)  # RAW: reposition targets
        # Clamp drive targets into the building envelope: an estimate outside
        # the walls (small's victim4, 3 m off and beyond the NE wall) is
        # physically unreachable — robots burned 4×40 s timeouts marching at
        # it. Clamped, the waypoint parks the robot inside the nearest room
        # where the arrival scan can actually see the victim.
        vids = [self._clamp_to_bbox(x, y) for x, y in all_waypoints]
        self._video_targets = list(vids)     # clamped: split + assist pool
        if vids and self._nav_wall is not None and len(vids) <= 6:
            assign = self._balanced_split(vids)
            my_wps = assign.get(self.robot_id, [])
            print(f"[{self.robot_id}] Balanced split: "
                  + " | ".join(f"{r}: " + ", ".join(f"({x:.1f},{y:.1f})" for x, y in g)
                               for r, g in assign.items()))
        else:
            my_wps = self._nn_order(self._select_waypoints(vids), (sx, sy))
        self._n_video_wps = len(my_wps)
        last   = my_wps[-1] if my_wps else (sx, sy)
        sweep  = self._nn_order(self._sweep_grid(), last)
        return my_wps + sweep

    def _bbox_reliable(self):
        """The flyover bbox is only trustworthy if it reaches the origin mat
        (the takeoff corner is part of the building by construction) and spans
        a plausible building. Large world fails this: its flyover walls are
        shifted ~5 m, the bbox starts at x≈5.7, and clamping dragged GOOD
        western estimates eastward while the sweep clip starved coverage."""
        if self._flyover_bbox is None:
            return False
        bx0, bx1, by0, by1 = self._flyover_bbox
        if bx1 - bx0 < 4.0 or by1 - by0 < 4.0:
            return False
        return (bx0 <= 1.5 and by0 <= 1.5 and bx1 >= -1.5 and by1 >= -1.5)

    def _envelope(self, slack=None):
        """Generous drivable envelope: the flyover bbox UNIONED with the origin
        mat, plus slack. The bbox is only metre-accurate — small world's starts
        1.4 m east of the true west wall, so a tight fence herded a robot that
        was legitimately inside the building (29/07: robot2, 0 victims). Slack
        must exceed the extraction's positional error."""
        if slack is None:
            slack = GEOFENCE_SLACK_M
        bx0, bx1, by0, by1 = self._flyover_bbox
        return (min(bx0, 0.0) - slack, max(bx1, 0.0) + slack,
                min(by0, 0.0) - slack, max(by1, 0.0) + slack)

    def _outside_envelope(self):
        """True when this robot has strayed well beyond the building envelope.
        Only meaningful for a trustworthy bbox; never fires without one."""
        if not self._bbox_reliable():
            return False
        x0, x1, y0, y1 = self._envelope()
        return not (x0 <= self.x <= x1 and y0 <= self.y <= y1)

    def _clamp_to_bbox(self, x, y, margin=0.4):
        """Clamp a point into the flyover building envelope — only when the
        envelope passes the reliability gate."""
        if not self._bbox_reliable():
            return (x, y)
        bx0, bx1, by0, by1 = self._flyover_bbox
        return (min(max(x, bx0 + margin), bx1 - margin),
                min(max(y, by0 + margin), by1 - margin))

    def _balanced_split(self, vids):
        """PLAN §3.1–3.2: divide victims into two groups (sizes differing ≤1)
        minimising total soft-cost A* route length, each robot's route anchored
        on its furthest victim (crow flies, visited last) with the intermediate
        order brute-forced by total A* length. Deterministic — both robots
        compute the identical assignment from START_POS and the startup grid."""
        from itertools import combinations, permutations
        starts = {r: START_POS[r] for r in ("robot1", "robot2")}

        def order_group(start, group):
            if not group:
                return [], 0.0
            anchor = max(group, key=lambda p: math.hypot(p[0] - start[0],
                                                         p[1] - start[1]))
            rest = [p for p in group if p is not anchor]
            best_seq, best_cost = None, float('inf')
            for perm in permutations(rest):
                seq, cost, prev = list(perm) + [anchor], 0.0, start
                for p in seq:
                    cost += self._route_cost(prev, p)
                    prev  = p
                if cost < best_cost:
                    best_seq, best_cost = seq, cost
            return best_seq, best_cost

        n = len(vids)
        best = None
        for k in sorted({n // 2, (n + 1) // 2}):
            for combo in combinations(range(n), k):
                g1 = [vids[i] for i in combo]
                g2 = [vids[i] for i in range(n) if i not in combo]
                s1, c1 = order_group(starts["robot1"], g1)
                s2, c2 = order_group(starts["robot2"], g2)
                if best is None or c1 + c2 < best[0] - 1e-9:
                    best = (c1 + c2, s1, s2)
        return {"robot1": best[1], "robot2": best[2]}

    def _select_waypoints(self, all_waypoints):
        """Fallback split (no nav grid): distance-rank video waypoints from
        this robot's start; interleave so robot1 takes even ranks and robot2
        odd ranks (disjoint targets)."""
        if not all_waypoints:
            return []
        sx, sy = START_POS.get(self.robot_id, (-0.375, 0.0))
        ranked = sorted(all_waypoints, key=lambda p: math.hypot(p[0] - sx, p[1] - sy))
        return ranked[0::2] if self.robot_id == "robot1" else ranked[1::2]

    def _sweep_grid(self):
        """
        Generic sweep grid: rows SWEEP_STEP_M apart covering a
        SWEEP_HALF_EXTENT_M square, robot1 covering the north half (y >= 0)
        and robot2 the south half (y <= 0) so the two robots don't retread
        each other's ground.
        """
        cols = np.arange(-SWEEP_HALF_EXTENT_M, SWEEP_HALF_EXTENT_M + 1e-6, SWEEP_STEP_M)
        if self.robot_id == "robot1":
            rows = np.arange(0.0, SWEEP_HALF_EXTENT_M + 1e-6, SWEEP_STEP_M)
        else:
            rows = np.arange(0.0, -SWEEP_HALF_EXTENT_M - 1e-6, -SWEEP_STEP_M)
        pts = [(float(x), float(y)) for y in rows for x in cols]
        # Victims are inside the building; sweep waypoints outside its walls
        # only march robots into open ground (18:02 run: robot2 walked 30 m
        # off-map on sweep points). Clip the sweep to the flyover wall bbox +
        # margin — the flyover map is too misaligned to route on, but its
        # overall extent is still a serviceable building envelope.
        if getattr(self, "_flyover_bbox", None):
            bx0, bx1, by0, by1 = self._flyover_bbox
            # The takeoff mat is part of the building by construction, so the
            # envelope must reach the origin. A bbox that doesn't (large: its
            # flyover walls sit ~5 m shifted) is under-covering — union it
            # with the origin and widen the margin instead of starving the
            # sweep (previous run: only 24+12 sweep waypoints survived).
            m = 1.0 if self._bbox_reliable() else 3.0
            x0, x1 = min(bx0, 0.0) - m, max(bx1, 0.0) + m
            y0, y1 = min(by0, 0.0) - m, max(by1, 0.0) + m
            inside = [(x, y) for (x, y) in pts if x0 <= x <= x1 and y0 <= y <= y1]
            if inside:
                pts = inside
        return pts

    @staticmethod
    def _nn_order(points, start_xy):
        """Greedy nearest-neighbour ordering from start_xy. Guarantees every
        leg goes to the closest remaining target — never a jump across the
        whole map — regardless of how the points were generated."""
        ordered, cx, cy, remaining = [], start_xy[0], start_xy[1], list(points)
        while remaining:
            nxt = min(remaining, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
            ordered.append(nxt)
            cx, cy = nxt
            remaining.remove(nxt)
        return ordered

    # ── Sensing ───────────────────────────────────────────────────────────

    def heading(self):
        """Current heading in radians (compass when available).

        World is Z-up ENU (East=+X, North=+Y, Up=+Z), so the compass's north
        vector lies in the X-Y plane: (cx, cy) = (sin ψ, cos ψ) for body yaw ψ
        measured CCW from world +X. Heading is therefore atan2(cx, cy) — the
        same convention the flyover IMU integration uses (atan2(comp_x,
        comp_y)). The previous atan2(cx, cz) used the vertical component (≈0),
        pinning heading near ±90° and making the robot dead-reckon in a frame
        rotated ~90° from the world.
        """
        if self._compass:
            cx, cy, _ = self._compass.getValues()
            return math.atan2(cx, cy)
        return self.theta

    def _min_front_range(self):
        """Minimum finite range in the front 120° lidar sector."""
        if not self._lidar:
            return float('inf')
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            # Webots can return a NULL buffer on the very first simulation step
            # for the second robot controller — treat as no obstacle.
            return float('inf')
        n      = len(ranges)
        front  = ranges[n // 3: 2 * n // 3]
        finite = [r for r in front if r < 1e5]
        return min(finite) if finite else float('inf')

    def _min_narrow_range(self):
        """Minimum finite range in a narrow (~±20°) cone dead ahead — used to
        decide whether the robot can drive straight (e.g. through a doorway)
        even when the wider front sector clips a door frame or nearby furniture."""
        if not self._lidar:
            return float('inf')
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            return float('inf')
        n = len(ranges)
        c = n // 2                      # front is the centre of the 360° scan
        w = max(1, int(n * 0.055))      # ~±20°
        seg = ranges[c - w:c + w]
        finite = [r for r in seg if r < 1e5]
        return min(finite) if finite else float('inf')

    def _clear_side(self):
        """Return 'left' or 'right' — whichever side has more clearance."""
        if not self._lidar:
            return 'right'
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            return 'right'
        n         = len(ranges)
        left_fin  = [r for r in ranges[:n // 3]     if r < 1e5]
        right_fin = [r for r in ranges[2 * n // 3:] if r < 1e5]
        left_min  = min(left_fin,  default=10.0)
        right_min = min(right_fin, default=10.0)
        return 'left' if left_min > right_min else 'right'

    def _side_range(self, side):
        """Minimum lidar range in the left or right 60-degree sector."""
        if not self._lidar:
            return float('inf')
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            return float('inf')
        n = len(ranges)
        seg = ranges[:n // 3] if side == 'left' else ranges[2 * n // 3:]
        fin = [r for r in seg if r < 1e5]
        return min(fin, default=float('inf'))

    def _range_at_bearing(self, beta, half_deg=20.0):
        """Minimum lidar range in a sector centred on RELATIVE bearing `beta`
        (radians, positive = left of the nose).

        The painting loop defines beam i's relative bearing as
        `pi - 2*pi*i/n`, so the inverse mapping is `i = n*(pi - beta)/(2*pi)`,
        taken modulo n. Used by the Bug2 leave test to ask the one question that
        matters while following a wall: is there now an open lane toward the
        goal — i.e. have we arrived at the doorway?"""
        if not self._lidar:
            return float('inf')
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            return float('inf')
        n = len(ranges)
        if n == 0:
            return float('inf')
        centre = int(round(n * (math.pi - beta) / (2.0 * math.pi))) % n
        half   = max(1, int(n * (half_deg / 360.0)))
        best   = float('inf')
        for k in range(centre - half, centre + half + 1):
            r = ranges[k % n]
            if r < 1e5:
                best = min(best, r)
        return best

    def _wf_should_leave(self, tx, ty, dist, now):
        """True when following has served its purpose (proper Bug2 leave test).

        Three ways to be done: (a) we are close enough that the planner can
        finish the job; (b) we are measurably nearer the goal than at the hit
        point AND the lidar shows an open lane toward it — a doorway; (c) we
        have come back to the hit point, so the surface encloses us and no
        amount of further following will help."""
        if dist < WF_HANDBACK_M:
            return True
        if dist < self._wf_hit_dist - WF_LEAVE_GAIN_M:
            beta = math.atan2(ty - self.y, tx - self.x) - self.theta
            beta = math.atan2(math.sin(beta), math.cos(beta))
            if abs(beta) < math.radians(WF_LEAVE_CONE_DEG):
                lane = self._range_at_bearing(beta)
                if lane > min(dist, WF_LEAVE_CLEAR_M):
                    return True
        if (self._wf_hit is not None and now - self._wf_started > 12.0
                and math.hypot(self.x - self._wf_hit[0],
                               self.y - self._wf_hit[1]) < WF_LOOP_M):
            print(f"[{self.robot_id}] Followed the surface back to where it was "
                  f"met — {dist:.1f} m from ({tx:.1f},{ty:.1f}) is enclosed")
            return True
        return False

    def _wall_follow_step(self):
        """Hold the followed surface at WF_KEEP_M on self._wf_side and drive
        forward, rotating away when the front is blocked. Bang-bang control
        with a 0.05 m dead-band: enough to traverse a wall to its doorway
        without knowing any geometry."""
        front = min(self._min_front_range(), self._ir_front_m())
        d     = self._side_range(self._wf_side)
        v     = MAX_SPEED * WF_SPEED_F
        turn_away = (self._wf_side == 'right')     # positive bias steers left
        if front < 0.35:                           # corner: rotate away
            w = MAX_SPEED * 0.35
            self.set_speeds(-w if turn_away else w, w if turn_away else -w)
            return
        # set_speeds(left, right) with bias>0 -> left slower -> turns LEFT.
        # Steering TOWARD a wall on the right therefore needs bias<0, hence the
        # per-side sign.
        sign = -1.0 if self._wf_side == 'right' else 1.0
        if not math.isfinite(d):
            bias = sign * 0.35                     # lost it: curve back toward
        elif d > WF_KEEP_M + 0.05:
            bias = sign * 0.25                     # too far: steer toward it
        elif d < WF_KEEP_M - 0.05:
            bias = -sign * 0.25                    # too close: steer away
        else:
            bias = 0.0
        self.set_speeds(v * (1.0 - bias), v * (1.0 + bias))

    def obstacle_ahead(self, threshold=0.6):
        return self._min_front_range() < threshold

    # ── Odometry ──────────────────────────────────────────────────────────

    def update_odometry(self):
        """Update (x, y, theta) from wheel encoders; use compass for heading when available."""
        fl = self._enc["fl"].getValue()
        fr = self._enc["fr"].getValue()

        if self._prev_fl is None:
            self._prev_fl, self._prev_fr = fl, fr
            return

        d_fl = (fl - self._prev_fl) * WHEEL_RADIUS
        d_fr = (fr - self._prev_fr) * WHEEL_RADIUS
        self._prev_fl, self._prev_fr = fl, fr

        dist = (d_fl + d_fr) / 2.0
        d_th = (d_fr - d_fl) / AXLE_LENGTH

        # Compass is authoritative when available; encoder integration otherwise
        if self._compass:
            self.theta = self.heading()
        else:
            self.theta += d_th

        self.x += dist * math.cos(self.theta)
        self.y += dist * math.sin(self.theta)

    # ── Actuation ─────────────────────────────────────────────────────────

    def set_speeds(self, left, right):
        left  = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        # Slew-rate limit: ramp commands instead of stepping them (see
        # SLEW_RAMP_S). Braking/reversing still engages within one ramp.
        step = MAX_SPEED * (self.timestep / 1000.0) / SLEW_RAMP_S
        left  = max(self._cmd_l - step, min(self._cmd_l + step, left))
        right = max(self._cmd_r - step, min(self._cmd_r + step, right))
        self._cmd_l, self._cmd_r = left, right
        self._motors["fl"].setVelocity(left)
        self._motors["rl"].setVelocity(left)
        self._motors["fr"].setVelocity(right)
        self._motors["rr"].setVelocity(right)

    def stop(self):
        self.set_speeds(0, 0)

    # ── Communication ─────────────────────────────────────────────────────

    def report_victim(self):
        """
        Send a victim-found message to the supervisor and broadcast to partner.

        Policy: gate then report HIGH. Callers must only invoke this once a
        victim is confirmed AND in range, so we always send a single fixed
        HIGH_CONFIDENCE rather than a marginal per-frame value.
        """
        if not self._sup_emitter:
            return
        try:
            # The sim ends the moment the LAST victim is reported — make sure
            # the learned-map deliverable is on disk before this send.
            self._write_learned_deliverable()
        except Exception:
            pass
        msg = {
            "timestamp":         self.robot.getTime(),
            "robot_id":          self.robot_id,
            "position":          [self.x, self.y, 0.0],
            "victim_found":      True,
            "victim_confidence": HIGH_CONFIDENCE,
        }
        self._sup_emitter.send(json.dumps(msg).encode())
        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)
        print(f"[{self.robot_id}] Victim reported @ ({self.x:.2f}, {self.y:.2f})  "
              f"conf={HIGH_CONFIDENCE:.2f}")

    def _already_reported_nearby(self, x, y, radius=0.5):
        """True if any confirmed victim (ours or partner's) is within radius metres.
        0.5 m: prevents exact double-reports while allowing the streaming check to
        fire a TRUE verdict when the robot is 0.3 m from victim1/3 bodies (which are
        0.70/0.73 m from their markers — within the 1.0 m scoring radius)."""
        return any(math.hypot(x - rx, y - ry) < radius
                   for rx, ry in self._reported_positions)

    def _broadcast_victim(self, x, y):
        """Tell the partner robot about this confirmed victim position."""
        if not self._squad_tx:
            return
        self._squad_tx.send(json.dumps({"type": "victim", "x": x, "y": y}).encode())

    def _broadcast_failed(self, x, y):
        """Tell the partner we have abandoned a target as unreachable."""
        if self._squad_tx:
            self._squad_tx.send(
                json.dumps({"type": "fail", "x": x, "y": y}).encode())

    def _broadcast_position(self):
        """Broadcast position + current video target to the partner robot.
        The target lets partner-assist skip the waypoint we are actively
        working (29/07 small run: robot2 adopted the victim robot1 was
        mid-homing on and both fought over it while victim1 went unvisited)."""
        if not self._squad_tx:
            return
        msg = {"type": "pos", "x": self.x, "y": self.y}
        if self.target_idx < min(self._n_video_wps, len(self._my_waypoints)):
            tx, ty = self._my_waypoints[self.target_idx]
            msg["tx"], msg["ty"] = tx, ty
        self._squad_tx.send(json.dumps(msg).encode())

    def _read_squad_messages(self):
        """Process incoming messages from the partner robot."""
        if not self._squad_rx:
            return
        while self._squad_rx.getQueueLength() > 0:
            try:
                data = self._squad_rx.getString()
                msg  = json.loads(data)
                if msg.get("type") == "victim":
                    vx, vy = float(msg["x"]), float(msg["y"])
                    if not self._already_reported_nearby(vx, vy):
                        self._reported_positions.append((vx, vy))
                        print(f"[{self.robot_id}] Partner confirmed victim @ "
                              f"({vx:.2f}, {vy:.2f})")
                elif msg.get("type") == "fail":
                    self._partner_failed.append((float(msg["x"]),
                                                 float(msg["y"])))
                elif msg.get("type") == "pos":
                    self._partner_x = float(msg["x"])
                    self._partner_y = float(msg["y"])
                    if "tx" in msg:
                        self._partner_target = (float(msg["tx"]),
                                                float(msg["ty"]))
                    else:
                        self._partner_target = None
            except (json.JSONDecodeError, KeyError, ValueError):
                pass
            self._squad_rx.nextPacket()

    # ── Camera victim detection ───────────────────────────────────────────

    def _camera_victim_confidence(self):
        """
        Return (confidence, min_depth_m, x_offset) for a likely victim.

        x_offset is the victim blob's horizontal position in the frame as a
        fraction in [-1, +1] (0 = centred, + = to the right) — used to steer
        the robot toward the victim while homing.

        Colour-segments skin tones and bright clothing in HSV.  If a depth
        camera is available, the detected region must be within 4 m — this
        rejects false positives from distant coloured objects and scales the
        confidence score by proximity.  min_depth_m is the nearest measured
        range to the detected region (0.0 when depth is unavailable, so the
        proximity gate falls back to confidence/odometry only).
        """
        if not self._camera:
            return 0.0, float('inf'), 0.0

        try:
            img = self._camera.getImage()
        except Exception:
            return 0.0, float('inf'), 0.0
        if not img:
            return 0.0, float('inf'), 0.0

        w   = self._camera.getWidth()
        h   = self._camera.getHeight()
        arr = np.frombuffer(img, np.uint8).reshape((h, w, 4))
        bgr = arr[:, :, :3]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Skin tones (warm tones: orange-yellow, lower saturation)
        mask  = cv2.inRange(hsv, np.array([ 5, 15,  60]), np.array([25, 200, 255]))
        # Colourful clothing — exclude pure red H<10 & H>165: the robot chassis
        # is bright red (H≈0, S>160) and triggers false positives when visible.
        mask |= cv2.inRange(hsv, np.array([10, 70,  60]), np.array([165, 255, 230]))
        k     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

        # Denim gate: every Webots victim model wears blue denim (the flyover
        # detector relies on the same invariant), while lounges/tables/kitchen
        # clutter carry warm colours but no denim. Ground-truth false positives
        # in mission runs were all furniture — require a visible denim patch
        # before treating the colour blob as a victim at all.
        denim = cv2.inRange(hsv, np.array([95, 35, 25]), np.array([135, 255, 255]))
        denim = cv2.morphologyEx(denim, cv2.MORPH_OPEN,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        # Largest CONNECTED denim blob. Counting raw denim pixels let speckle
        # scattered across the frame (sky through a window, blue highlights)
        # pass the gate; a victim's denim is one contiguous region.
        n_lab, labels, stats, cents = cv2.connectedComponentsWithStats(denim, 8)
        if n_lab <= 1:
            return self._vision_reject("no-denim")
        best      = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        blob_frac = float(stats[best, cv2.CC_STAT_AREA]) / (w * h)
        if blob_frac < DENIM_MIN_COMPONENT_FRAC:
            return self._vision_reject("denim-blob-small", blob=blob_frac)
        blob = (labels == best).astype(np.uint8)

        ratio = cv2.countNonZero(mask) / (w * h)
        if ratio <= 0.04:
            return self._vision_reject("colour-ratio", blob=blob_frac, ratio=ratio)

        base_conf = min(0.90, ratio * 15)
        # Horizontal offset for steering (fraction of half-width, [-1,1]), taken
        # from the denim blob's centroid — that's the victim itself, not the
        # furniture that may share the warm-colour mask.
        x_offset  = (float(cents[best][0]) / w - 0.5) * 2.0
        # Default inf so the depth gate FAILS when the depth camera returns
        # NaN/inf for a region (distant/out-of-range pixels).
        min_depth = float('inf')
        med_h = low_h = float('nan')
        width_m = float('nan')

        # Depth + geometry refinement over the denim BLOB only.
        if self._depth_camera:
            try:
                dw  = self._depth_camera.getWidth()
                dh  = self._depth_camera.getHeight()
                depth_arr = np.array(self._depth_camera.getRangeImage(),
                                     dtype=np.float32).reshape((dh, dw))
                mask_d = cv2.resize(blob, (dw, dh),
                                    interpolation=cv2.INTER_NEAREST).astype(bool)
                rows   = np.nonzero(mask_d)[0]
                d      = depth_arr[mask_d]
                finite = np.isfinite(d) & (d > 0.1)
                if not finite.any():
                    # Denim with no finite depth is sky or something beyond the
                    # range finder — never a victim we can drive to.
                    return self._vision_reject("denim-no-depth", blob=blob_frac,
                                               ratio=ratio)
                d, rows = d[finite], rows[finite]
                # MEDIAN, not min: np.min let a single stray blue pixel on a
                # nearby surface trip the <0.80 m report gate from metres away.
                min_depth = float(np.median(d))
                if min_depth > 4.0:
                    return self._vision_reject("denim-far", blob=blob_frac,
                                               ratio=ratio, depth=min_depth)

                # Height above the floor: level camera at CAM_HEIGHT_M, pixel
                # row → elevation angle → h = cam_h + z·tan(elev).
                hf   = float(self._depth_camera.getFov())
                vf   = 2.0 * math.atan(math.tan(hf * 0.5) * dh / float(dw))
                f_y  = (dh * 0.5) / math.tan(vf * 0.5)
                elev = np.arctan2((dh * 0.5 - rows), f_y)
                hgt  = CAM_HEIGHT_M + d * np.tan(elev)
                med_h = float(np.median(hgt))
                low_h = float(np.percentile(hgt, 10))
                # A person's denim reaches down toward the floor; window glass
                # never does. Geometry, not world knowledge — safe on unseen maps.
                if med_h > VICTIM_DENIM_MAX_H_M or low_h > VICTIM_DENIM_LOW_H_M:
                    return self._vision_reject("denim-too-high", blob=blob_frac,
                                               ratio=ratio, depth=min_depth,
                                               med_h=med_h, low_h=low_h)

                # PHYSICAL denim area (px × depth² / focal²) — unlike the blob
                # fraction this does not scale away with distance, so a small
                # blue object (the 17:32 decoy: ~0.007 m² at 0.81 m) can never
                # impersonate trousers no matter how close the robot gets.
                try:
                    fx = (w * 0.5) / math.tan(self._camera.getFov() * 0.5)
                except Exception:
                    fx = float(w)
                area_m2 = blob_frac * w * h * (min_depth ** 2) / (fx * fx)
                if area_m2 < VICTIM_DENIM_MIN_AREA_M2:
                    return self._vision_reject("denim-area-small", blob=blob_frac,
                                               ratio=ratio, depth=min_depth,
                                               med_h=med_h, low_h=low_h)

                # Apparent WIDTH of the denim blob in metres. This tells us which
                # way the body is lying relative to us, which decides whether
                # the SCORED point (the waist centroid, projected to the floor)
                # is within the supervisor's 1.0 m radius when we report:
                #   broadside  → blob ~1.0-1.7 m wide, centroid ≈ depth + 0.2 m
                #   end-on     → blob ~0.3-0.5 m wide, centroid ≈ depth + 0.85 m
                # Measured on large_world 30/07: a robot reported at 0.7 m depth
                # while the centroid was 1.43 m away — verdict False, confidence
                # 0.99 → 0.511, AND its own report then blocked it from closing
                # on the real victim it was still approaching.
                blob_w_px = float(stats[best, cv2.CC_STAT_WIDTH])
                width_m = blob_w_px * min_depth / fx

                proximity = max(0.5, 1.0 - min_depth / 5.0)
                base_conf = min(0.95, base_conf * (1.0 + proximity * 0.5))
            except Exception as exc:                      # never crash the mission
                self._last_vision["why"] = "depth-error:%s" % exc

        self._last_vision = {"why": "candidate", "conf": base_conf,
                             "depth": min_depth, "blob": blob_frac,
                             "ratio": ratio, "med_h": med_h, "low_h": low_h,
                             "width_m": width_m}
        return base_conf, min_depth, x_offset

    def _vision_reject(self, why, **fields):
        """Record why the detection gate closed, and return the null detection.
        Purely diagnostic bookkeeping — the return value is unchanged."""
        rec = {"why": why, "conf": 0.0, "depth": fields.pop("depth", float('inf'))}
        rec.update(fields)
        self._last_vision = rec
        return 0.0, rec["depth"], 0.0

    def _vision_telemetry(self):
        """Throttled one-line dump of the detection gate's internal state, so a
        single Webots run reveals whether the gate ever opens on a real victim
        and, when it doesn't, which condition closed it. DEBUG_VISION only."""
        if not DEBUG_VISION:
            return
        now = self.robot.getTime()
        if now < self._dbg_next_t:
            return
        self._dbg_next_t = now + DEBUG_VISION_EVERY_S
        self._camera_victim_confidence()      # refresh _last_vision (read-only)
        v     = self._last_vision
        state = "HOME" if self._homing else ("SCAN" if self._scanning else "DRIVE")
        if self.target_idx < len(self._my_waypoints):
            tx, ty = self._my_waypoints[self.target_idx]
        else:
            tx = ty = float('nan')
        print("[%s] t=%6.1f %-5s pos=(%6.2f,%6.2f) wp%-3d=(%6.2f,%6.2f) "
              "conf=%.2f depth=%6.2f blob=%.4f ratio=%.3f h_med=%5.2f h_low=%5.2f "
              "wid=%5.2f "
              "why=%-18s block=%s%s"
              % (self.robot_id, now, state, self.x, self.y, self.target_idx, tx, ty,
                 v.get("conf", 0.0), v.get("depth", float('inf')),
                 v.get("blob", 0.0), v.get("ratio", 0.0),
                 v.get("med_h", float('nan')), v.get("low_h", float('nan')),
                 v.get("width_m", float('nan')),
                 v.get("why", "-"), self._home_block,
                 " FP-ZONE" if self._victim_false_positive_zone() else ""))

    def _victim_false_positive_zone(self):
        """True near the origin mat or the partner robot — both trigger HSV
        false positives (building textures, the partner's red chassis)."""
        if math.hypot(self.x, self.y) < 3.0:
            return True
        px = self._partner_x if self._partner_x is not None else -0.375
        py = self._partner_y if self._partner_y is not None else (
            0.0 if self.robot_id == "robot1" else 0.375)
        return math.hypot(self.x - px, self.y - py) < 2.5

    def _maybe_start_homing(self):
        """Begin homing when the camera sees a solid victim candidate and we're
        clear of the mat/partner false-positive zones and any prior report."""
        if self._victim_false_positive_zone():
            self._home_block = "fp-zone"
            return
        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
        conf, depth, _ = self._camera_victim_confidence()
        # Require a FINITE denim depth in range: a denim patch with no depth
        # reading is a victim on the far side of a wall/doorway — homing at it
        # just rams the wall (seen in mission logs).
        if conf >= APPROACH_ENTER_CONF and math.isfinite(depth) \
           and depth <= APPROACH_ENTER_M:
            self._homing          = True
            self._home_start_time = self.robot.getTime()
            self._home_lost_ticks = 0
            self._scanning        = False
            self._report_streak   = 0
            self._home_best_depth = float('inf')
            self._home_best_t     = self.robot.getTime()
            self._home_block      = "-"
            return
        self._home_block = "gate"

    def _approach_victim(self):
        """Drive toward the detected victim until within APPROACH_STOP_M, then
        report. Aborts on timeout or if the victim is lost from view."""
        now = self.robot.getTime()

        # ── PLAN §3.5: repositioning toward a CSV estimate before reporting ──
        if self._repositioning is not None:
            ex, ey = self._repositioning
            d = math.hypot(ex - self.x, ey - self.y)
            stalled = False
            if self._repo_prev is None:
                self._repo_prev, self._repo_stall_t = (self.x, self.y), now
            elif math.hypot(self.x - self._repo_prev[0],
                            self.y - self._repo_prev[1]) > 0.15:
                self._repo_prev, self._repo_stall_t = (self.x, self.y), now
            elif now - self._repo_stall_t > 2.5:
                stalled = True                    # furniture in the way — report here
            if d <= REPOSITION_STOP_M or now > self._repo_deadline or stalled:
                self.stop()
                self._repositioning = None
                if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                    self.report_victim()
                if self.target_idx < len(self._my_waypoints):
                    tx, ty = self._my_waypoints[self.target_idx]
                    at_target = math.hypot(self.x - tx, self.y - ty) <= 2.5
                else:
                    at_target = False
                self._end_homing(advance=at_target)
                return
            self._drive_toward(ex, ey, now)
            return

        conf, depth, offset = self._camera_victim_confidence()

        if self._home_start_time is not None and \
           now - self._home_start_time > APPROACH_TIMEOUT_S:
            self._end_homing(advance=False)      # keep the waypoint; cooldown
            return

        # Close enough and still see it → park, confirm over several frames,
        # then report (once). A SINGLE frame satisfying the gate is what
        # produced the standing false positive; holding station for
        # REPORT_STREAK_N ticks costs a fraction of a second and is the
        # cheapest insurance available against poisoning the confidence score.
        if math.isfinite(depth) and depth <= APPROACH_STOP_M and conf >= 0.15:
            # Creep rather than freeze while the confirmation streak runs:
            # every extra centimetre closed is margin against the supervisor's
            # 1.0 m TRUE-position gate (and against our own odometry drift).
            self._report_streak += 1
            if self._report_streak < REPORT_STREAK_N:
                if depth > 0.62:
                    self.set_speeds(MAX_SPEED * 0.10, MAX_SPEED * 0.10)
                else:
                    self.stop()
                return
            self.stop()
            # Marker-offset countermeasure: the CSV estimate is a better proxy
            # for the scored marker than the body we are parked against. If an
            # unreported estimate is nearby, walk to it and report from there.
            est, d_est = self._nearest_estimate()
            if est is not None and REPOSITION_MIN_M < d_est <= REPOSITION_MAX_M:
                self._repositioning   = est
                self._repo_deadline   = now + REPOSITION_TIMEOUT_S
                self._repo_prev       = None
                self._home_start_time = now
                print(f"[{self.robot_id}] Denim locked — repositioning toward "
                      f"estimate ({est[0]:.2f},{est[1]:.2f}) before reporting")
                return
            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
            # Retire the waypoint ONLY when the find is at the waypoint we were
            # driving to. An opportunistic sighting en route (or a false one)
            # must not delete a real target from the itinerary — the old
            # unconditional advance=True did exactly that.
            if self.target_idx < len(self._my_waypoints):
                tx, ty   = self._my_waypoints[self.target_idx]
                at_target = math.hypot(self.x - tx, self.y - ty) <= 2.5
            else:
                at_target = False
            self._end_homing(advance=at_target)
            return
        self._report_streak = 0

        # Physical stall: depth stopped improving while we drive at the victim
        # (21:51 run: robot wedged against a bed rail the lidar plane cannot
        # see). Abort with a reverse burst instead of grinding indefinitely.
        if math.isfinite(depth) and depth < 1.5:
            if depth < self._home_best_depth - 0.05:
                self._home_best_depth = depth
                self._home_best_t     = now
            elif now - self._home_best_t > 3.0:
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                self._end_homing(advance=False)
                return

        # Lost the victim → rotate to reacquire; abort if lost too long
        if conf < 0.15:
            self._home_lost_ticks += 1
            self.set_speeds(-SCAN_TURN_SPEED, SCAN_TURN_SPEED)
            if self._home_lost_ticks > int(APPROACH_LOST_S * 1000 / self.timestep):
                self._end_homing(advance=False)
            return
        self._home_lost_ticks = 0

        # A wall (not the victim) directly ahead → sidestep toward clearer side
        front = self._min_front_range()
        if front < 0.25 and not (math.isfinite(depth) and depth < 0.7):
            if self._clear_side() == 'left':
                self.set_speeds(0.0, APPROACH_SPEED)
            else:
                self.set_speeds(APPROACH_SPEED, 0.0)
            return

        # Steer toward the blob; slow down as we close in
        turn  = HOME_TURN_GAIN * offset
        speed = APPROACH_SPEED
        if math.isfinite(depth):
            speed *= max(0.4, min(1.0, depth / 2.0))
        self.set_speeds(speed + turn, speed - turn)

    def _nearest_estimate(self):
        """Nearest flyover victim estimate not already covered by a report."""
        best, bd = None, float('inf')
        for (ex, ey) in self._video_estimates:
            if self._already_reported_nearby(ex, ey, radius=1.0):
                continue
            d = math.hypot(ex - self.x, ey - self.y)
            if d < bd:
                best, bd = (ex, ey), d
        return best, bd

    def _end_homing(self, advance=False):
        """Leave homing state. advance=True only after a successful report —
        a failed homing keeps the waypoint and sets a cooldown so the robot
        resumes its route instead of re-homing at the same blocked sighting."""
        self._homing          = False
        self._home_start_time = None
        self._home_lost_ticks = 0
        self._report_streak   = 0
        self._repositioning   = None
        self.stop()
        if advance and self.target_idx < len(self._my_waypoints):
            self.target_idx    += 1
            self._wp_start_time = None
        if not advance:
            self._home_cooldown_until = self.robot.getTime() + 8.0

    # ── Global route planning (A* on the flyover wall map) ────────────────

    def _load_nav_grid(self):
        """LIDAR-FIRST routing grid. The flyover wall map measured misaligned
        by 0.7–1.5 m in a per-world random direction (VO drift; small −0.25,−1.30 /
        medium −0.25,+0.65 / large −1.45,+0.15 m vs ground truth) — walls that
        wrong seal real doorways and open fake ones, causing the darting-at-
        walls behaviour. So the ROUTING grid starts EMPTY and is populated
        exclusively by the lidar learner (startup spin + 5 m horizon while
        driving). The flyover contributes only its wall bounding box (sweep
        containment) — its victim estimates are handled separately and remain
        trusted."""
        self._nav_wall  = None
        self._nav_cost  = None
        self._cost_dirty = True
        try:
            import prepare_mission as pm
            self._sim_logs = pm.SIM_LOGS
            N = int(round(2 * pm.GRID_HALF_M / NAV_RES))
            self._nav_wall  = np.zeros((N, N), np.uint8)
            self._nav_prior = np.zeros((N, N), np.uint8)   # deliverable = painted-only
            self._nav_seen  = np.zeros((N, N), np.uint8)
            self._nav_N     = N
            self._flyover_bbox = None
            self._flyover_wall = None
            path = os.path.join(pm.SIM_LOGS, "nav_grid.npy")
            if os.path.exists(path):
                grid = np.load(path)                    # 255=wall, origin @ centre
                # Downsample the flyover walls to NAV_RES and keep them ONLY to
                # judge, at the end of the mission, which wall map to submit.
                # They are deliberately NOT fed to the planner (see above).
                ff = max(1, int(round(NAV_RES / pm.MAP_SCALE)))
                n2 = grid.shape[0] // ff
                self._flyover_wall = (grid[:n2 * ff, :n2 * ff]
                                      .reshape(n2, ff, n2, ff)
                                      .max(axis=(1, 3)) > 0)
                ys, xs = np.nonzero(grid)
                if len(xs):
                    s = pm.MAP_SCALE
                    h = grid.shape[0] / 2.0
                    self._flyover_bbox = ((xs.min() - h) * s, (xs.max() - h) * s,
                                          (h - ys.max()) * s, (h - ys.min()) * s)
            print(f"[{self.robot_id}] Nav map: {N}x{N} @ {NAV_RES} m/px, "
                  f"lidar-first (empty prior; flyover bbox "
                  f"{'set' if self._flyover_bbox else 'none'})")
        except Exception as e:
            print(f"[{self.robot_id}] Nav map init failed ({e}) — reactive nav only")
            self._nav_wall = None

    def _cost_grid(self):
        """Traversal-cost grid derived from the wall grid: wall cells cost
        NAV_WALL_COST (finite — crossable when there is no better way), cells
        within NAV_INFLATE_M of a wall cost NAV_NEAR_COST (clearance by
        inflation only), everything else costs 1. Rebuilt lazily whenever the
        lidar learner edits the wall grid."""
        if self._nav_wall is None:
            return None
        if self._nav_cost is None or self._cost_dirty:
            inf_px = max(1, int(round(NAV_INFLATE_M / NAV_RES)))
            k      = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                               (2 * inf_px + 1, 2 * inf_px + 1))
            near   = cv2.dilate(self._nav_wall, k)
            cost   = np.ones(self._nav_wall.shape, dtype=np.float32)
            cost[near > 0]           = NAV_NEAR_COST
            cost[self._nav_wall > 0] = NAV_WALL_COST
            self._nav_cost   = cost
            self._cost_dirty = False
        return self._nav_cost

    def _w2c(self, x, y):
        """Mat-frame metres → (row, col). Origin at grid centre, +X→+col, +Y→-row."""
        n = self._nav_N
        return int(round(n / 2 - y / NAV_RES)), int(round(n / 2 + x / NAV_RES))

    def _c2w(self, r, c):
        n = self._nav_N
        return ((c - n / 2) * NAV_RES, (n / 2 - r) * NAV_RES)

    def _clamp_rc(self, rc):
        n = self._nav_N
        return (min(max(rc[0], 0), n - 1), min(max(rc[1], 0), n - 1))

    def _astar(self, s, g, hard=False):
        """Weighted A* between grid cells on the soft-cost grid.

        hard=False: every cell is traversable (walls just cost a lot), so a
        route always exists inside the budget; a huge cost still flags a
        suspect map (PLAN §1 self-check).

        hard=True: wall cells are IMPASSABLE. Used as the first attempt by
        _plan_path — a soft finite wall cost lets a short route through a wall
        beat a long route round it, which is what made the robots drive into
        walls. Blocking walls is also FASTER to search, not slower: raising
        NAV_WALL_COST instead (tried 300 on 30/07) weakens the unit-cost
        heuristic until A* degenerates to Dijkstra and the controller misses
        its timestep deadlines badly enough to hang a whole mission.
        Returns (cells, cost) or (None, inf)."""
        cost = self._cost_grid()
        if cost is None:
            return None, float('inf')
        blocked = (self._nav_wall > 0) if hard else None
        import heapq
        n = self._nav_N
        s, g = self._clamp_rc(s), self._clamp_rc(g)
        openh = [(0.0, s)]
        came, gsc = {}, {s: 0.0}
        found, expanded = False, 0
        while openh and expanded < NAV_ASTAR_BUDGET:
            _, cur = heapq.heappop(openh)
            if cur == g:
                found = True
                break
            expanded += 1
            cr, cc = cur
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = cr + dr, cc + dc
                    if not (0 <= nr < n and 0 <= nc < n):
                        continue
                    if blocked is not None and blocked[nr, nc] and (nr, nc) != g:
                        continue
                    step = math.hypot(dr, dc) * float(cost[nr, nc])
                    ng   = gsc[cur] + step
                    if (nr, nc) not in gsc or ng < gsc[(nr, nc)]:
                        gsc[(nr, nc)] = ng
                        came[(nr, nc)] = cur
                        h = math.hypot(nr - g[0], nc - g[1])   # admissible: min cost 1
                        heapq.heappush(openh, (ng + h, (nr, nc)))
        if not found:
            return None, float('inf')
        total = gsc[g]
        cells = [g]
        while g in came:
            g = came[g]
            cells.append(g)
        cells.reverse()
        return cells, total

    def _frontier_goal(self):
        """Nearest reachable FRONTIER: a cell we have observed to be free that
        borders unobserved space — i.e. the edge of this robot's knowledge.

        Driving frontiers instead of a fixed lattice is what lets the pair find
        rooms the flyover never revealed (medium_world's east room is reached
        through a doorway invisible from above; no preset waypoint can want to
        go there). It also guarantees the target is reachable-in-principle:
        frontier cells are free cells we have actually seen, unlike lattice
        points that may sit inside a wall or a sofa (29/07 medium run: robot1
        spent its whole mission timing out on two such points, 0 victims).
        Returns (x, y) or None when the reachable space is fully explored."""
        if self._nav_wall is None or self._nav_seen is None:
            return None
        seen    = self._nav_seen > 0
        free    = seen & (self._nav_wall == 0)
        unknown = ~seen
        # 4-neighbourhood of unknown space
        nb = np.zeros_like(unknown)
        nb[1:, :]  |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:]  |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        frontier = free & nb
        if not frontier.any():
            return None

        rows, cols = np.nonzero(frontier)
        xs, ys = [], []
        for r, c in zip(rows, cols):
            x, y = self._c2w(int(r), int(c))
            xs.append(x); ys.append(y)
        xs = np.asarray(xs); ys = np.asarray(ys)

        # Keep frontiers inside the drivable envelope (when the bbox is
        # trustworthy) so exploration cannot march out of the building.
        if self._bbox_reliable():
            x0, x1, y0, y1 = self._envelope()
            m = (xs >= x0) & (xs <= x1) & (ys >= y0) & (ys <= y1)
            xs, ys = xs[m], ys[m]
            if xs.size == 0:
                return None

        d = np.hypot(xs - self.x, ys - self.y)
        # Ignore frontiers we are practically standing on (sensor edge noise).
        keep = d > 1.0
        xs, ys, d = xs[keep], ys[keep], d[keep]
        if xs.size == 0:
            return None

        # Division of labour: leave frontiers the partner is closer to.
        if self._partner_x is not None:
            dp = np.hypot(xs - self._partner_x, ys - self._partner_y)
            mine = dp > d
            if mine.any():
                xs, ys, d = xs[mine], ys[mine], d[mine]

        order = np.argsort(d)
        tried = 0
        for i in order:
            gx, gy = float(xs[i]), float(ys[i])
            if any(math.hypot(gx - bx, gy - by) < 1.5
                   for bx, by in self._frontier_failed):
                continue
            if math.isfinite(self._route_cost((self.x, self.y), (gx, gy))):
                return (gx, gy)
            self._frontier_failed.append((gx, gy))
            tried += 1
            if tried >= FRONTIER_MAX_TRIES:      # keep the tick cheap
                break
        return None

    def _maybe_frontier_target(self, now):
        """In exploration mode (victim waypoints done), keep the current
        waypoint pointed at the nearest useful frontier. Falls through to the
        preset sweep lattice when the space is explored or no frontier is
        reachable. DISABLED by default; see EXPLORE_FRONTIERS."""
        if now < self._frontier_next_t:
            return
        self._frontier_next_t = now + FRONTIER_REPLAN_S
        goal = self._frontier_goal()
        if goal is None:
            return
        # Exploration targets are not victim estimates: stop the video-leg
        # requeue machinery from treating them as such (29/07: a frontier was
        # "requeued" and re-driven like a missed victim).
        self._n_video_wps = min(self._n_video_wps, self.target_idx)
        cur = (self._my_waypoints[self.target_idx]
               if self.target_idx < len(self._my_waypoints) else None)
        if cur is not None and math.hypot(cur[0] - goal[0],
                                          cur[1] - goal[1]) < 1.0:
            return                                # already heading there
        if self.target_idx < len(self._my_waypoints):
            self._my_waypoints[self.target_idx] = goal
        else:
            self._my_waypoints.append(goal)
        self._wp_start_time = None
        self._path          = None
        self._explore_count += 1
        if self._explore_count <= 3 or self._explore_count % 5 == 0:
            print(f"[{self.robot_id}] Exploring frontier "
                  f"({goal[0]:.1f},{goal[1]:.1f})")

    def _plan_path(self, gx, gy):
        """Soft-cost A* from the robot to (gx, gy). Returns a list of (x, y)
        sub-waypoints (~0.6 m apart), or None if no map / budget blown."""
        if self._nav_wall is None:
            return None
        s, g = self._w2c(self.x, self.y), self._w2c(gx, gy)
        # Prefer a route that does not cross known walls; only if none exists
        # (goal inside an unexplored room, or the robot boxed in by phantom
        # walls) fall back to the soft-cost search that may cut through one.
        cells = None
        if HARD_WALL_FIRST:
            cells, _ = self._astar(s, g, hard=True)
        if not cells:
            cells, _ = self._astar(s, g)
        if not cells:
            return None
        pts, last = [], None
        for (r, c) in cells:
            w = self._c2w(r, c)
            if last is None or math.hypot(w[0] - last[0], w[1] - last[1]) >= 0.6:
                pts.append(w)
                last = w
        pts.append(self._c2w(*cells[-1]))              # ensure the goal cell is last
        return pts or None

    def _route_cost(self, p, q):
        """Cost-weighted A* route length (grid units) between two mat-frame
        points — the metric PLAN §3.1 minimises when splitting victims. Cached;
        deterministic across both robots (depends only on the startup grid)."""
        key = (round(p[0], 2), round(p[1], 2), round(q[0], 2), round(q[1], 2))
        if key in self._route_cost_cache:
            return self._route_cost_cache[key]
        _, cost = self._astar(self._w2c(*p), self._w2c(*q))
        if not math.isfinite(cost):
            cost = (math.hypot(q[0] - p[0], q[1] - p[1]) + 1.0) * 50.0 / NAV_RES
        self._route_cost_cache[key] = cost
        self._route_cost_cache[(key[2], key[3], key[0], key[1])] = cost
        return cost

    def _update_map_from_lidar(self):
        """Two-way map learning (PLAN §3.4): endpoints of lidar rays paint
        walls INTO the wall grid; the free space each ray passes through erases
        phantom walls OUT of it. Painting invalidates the current path so the
        next tick re-plans around the wall we just met."""
        if self._nav_wall is None or not self._lidar:
            return
        if self._tilt_deg_now > LIDAR_TILT_MAX_DEG:
            return    # tilted lidar sweeps floor/ceiling — everything it
                      # "sees" is phantom (flipped robot painted a 0.003 map)
        now = self.robot.getTime()
        if now < self._lidar_map_next_t:
            return
        self._lidar_map_next_t = now + LIDAR_MAP_EVERY_S
        try:
            ranges = self._lidar.getRangeImage()
        except (ValueError, Exception):
            return
        n_beams = len(ranges)
        if n_beams == 0:
            return
        wall, N  = self._nav_wall, self._nav_N
        seen     = self._nav_seen
        changed  = painted = False
        step     = max(1, n_beams // LIDAR_MAP_BEAMS)
        for i in range(0, n_beams, step):
            r   = ranges[i]
            hit = r if r < 1e5 else float('inf')
            ang = self.theta + (math.pi - 2.0 * math.pi * i / n_beams)
            ca, sa = math.cos(ang), math.sin(ang)
            # Erase phantom walls along the free segment of the ray (stop well
            # short of the hit so we never rub out the wall we're looking at).
            limit = min(hit - 0.15, LIDAR_ERASE_MAX_M)
            d = 0.35
            while d < limit:
                rr, cc = self._w2c(self.x + d * ca, self.y + d * sa)
                if 0 <= rr < N and 0 <= cc < N:
                    seen[rr, cc] = 1              # lidar observed this cell
                    if wall[rr, cc]:
                        wall[rr, cc] = 0
                        changed = True
                d += NAV_RES
            # Paint the endpoint as wall — a 3x3 patch, not a single cell, so
            # a wall met head-on gains enough mass that crossing it costs more
            # than the doorway detour (single cells were crossable too cheaply:
            # the 18:02 run's robots kept nosing the same wall).
            if hit < LIDAR_PAINT_MAX_M:
                r0, c0 = self._w2c(self.x + hit * ca, self.y + hit * sa)
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r0 + dr, c0 + dc
                        if 0 <= rr < N and 0 <= cc < N:
                            seen[rr, cc] = 1
                            if not wall[rr, cc]:
                                wall[rr, cc] = 1
                                changed = painted = True
        if changed:
            self._cost_dirty = True
        if painted and now - self._last_plan_time > 1.5:
            self._path = None                          # re-plan around new walls

    # ── End-of-mission learned-map deliverable ─────────────────────────────

    def _learned_map_step(self):
        """Dump this robot's learned grid every LEARNED_SAVE_EVERY_S; robot1
        additionally merges both grids and rewrites map_estimate.png at
        LEARNED_WRITE_TIMES. Everything best-effort — a failure here must
        never touch the mission."""
        if self._nav_wall is None or not self._sim_logs:
            return
        now = self.robot.getTime()
        try:
            if now >= self._learned_next_t:
                self._learned_next_t = now + LEARNED_SAVE_EVERY_S
                np.savez(os.path.join(self._sim_logs,
                                      f"learned_{self.robot_id}.npz"),
                         wall=self._nav_wall, seen=self._nav_seen)
            if self.robot_id != "robot1":
                return
            for t_write in LEARNED_WRITE_TIMES:
                if now >= t_write and t_write not in self._learned_written:
                    self._learned_written.add(t_write)
                    ok = self._write_learned_deliverable()
                    print(f"[{self.robot_id}] Learned-map deliverable at "
                          f"t={now:.0f}s: {'written' if ok else 'kept flyover map'}")
        except Exception as exc:                       # never crash the mission
            print(f"[{self.robot_id}] learned-map step failed: {exc}")

    def _write_learned_deliverable(self):
        """Merge own + partner learned grids and rewrite map_estimate.png.

        Called at LEARNED_WRITE_TIMES (robot1) and immediately before EVERY
        victim report (either robot): the supervisor ends the simulation the
        instant the last victim is reported, so the deliverable must already
        be on disk when any potentially-final report is sent.
        write_learned_map refuses sparse grids (< LEARNED_MIN_WALL_PX), so
        early-mission reports keep the flyover map rather than degrading it."""
        if self._nav_wall is None or not self._sim_logs:
            return False
        partner_id = "robot2" if self.robot_id == "robot1" else "robot1"
        partner = os.path.join(self._sim_logs, f"learned_{partner_id}.npz")
        grids = [(self._nav_wall, self._nav_seen)]
        if os.path.exists(partner):
            z = np.load(partner)
            grids.append((z["wall"], z["seen"]))
        bbox_rc = None
        if self._flyover_bbox is not None:
            bx0, bx1, by0, by1 = self._flyover_bbox
            # Envelope must include the origin mat; widen when the flyover
            # bbox failed the reliability gate (misregistered extraction).
            m = 2.0 if self._bbox_reliable() else 4.0
            bx0, bx1 = min(bx0, 0.0), max(bx1, 0.0)
            by0, by1 = min(by0, 0.0), max(by1, 0.0)
            r0, c0 = self._w2c(bx0 - m, by1 + m)
            r1, c1 = self._w2c(bx1 + m, by0 - m)
            bbox_rc = (min(r0, r1), max(r0, r1),
                       min(c0, c1), max(c0, c1))
        return write_learned_map(self._sim_logs, self._nav_prior, grids,
                                 bbox_rc,
                                 flyover=getattr(self, '_flyover_wall', None))

    def _read_tilt(self):
        """Body tilt from vertical, degrees, from the accelerometer's gravity
        vector. 0 when the accelerometer is unavailable or mid free-fall."""
        if not self._accel:
            return 0.0
        try:
            ax, ay, az = self._accel.getValues()
        except Exception:
            return 0.0
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if math.isnan(g) or g < 1e-3:
            return 0.0
        return math.degrees(math.acos(max(-1.0, min(1.0, az / g))))

    def _ir_front_m(self):
        """Nearest reading of the two front IR sensors (0.053 m mount height —
        sees stairs/rubble below the lidar plane). inf when clear."""
        vals = []
        for nm in ("fl_range", "fr_range"):
            d = self._ir.get(nm)
            if d is None:
                continue
            try:
                v = d.getValue()
            except Exception:
                continue
            if not math.isnan(v) and 0.0 < v < IR_CLEAR_V:
                vals.append(v)
        return min(vals) if vals else float("inf")

    def _paint_low_obstacle(self, dist):
        """Mark a low obstacle (invisible to the lidar) into the nav grid so
        A* routes around it instead of retrying the same collision."""
        if self._nav_wall is None:
            return
        ox = self.x + math.cos(self.theta) * (dist + 0.15)
        oy = self.y + math.sin(self.theta) * (dist + 0.15)
        r, c = self._w2c(ox, oy)
        n = self._nav_N
        if 1 <= r < n - 1 and 1 <= c < n - 1:
            self._nav_wall[r - 1:r + 2, c - 1:c + 2] = 1
            self._nav_seen[r - 1:r + 2, c - 1:c + 2] = 1
            self._cost_dirty = True
            self._path = None

    def _drive_toward(self, gx, gy, now):
        """Forward-biased drive toward (gx, gy) with lidar avoidance + stuck-escape."""
        # Stuck detection
        if self._last_prog_time is None or now - self._last_prog_time >= STUCK_WINDOW_S:
            if self._last_prog_pos is not None and self._escape_ticks == 0:
                moved = math.hypot(self.x - self._last_prog_pos[0],
                                   self.y - self._last_prog_pos[1])
                if moved < STUCK_DIST_M:
                    self._escape_ticks = int(ESCAPE_S * 1000 / self.timestep)
                    self._path = None          # force a re-plan after escaping
            self._last_prog_pos  = (self.x, self.y)
            self._last_prog_time = now
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            if self._clear_side() == 'left':
                self.set_speeds(-MAX_SPEED * 0.35, -MAX_SPEED * 0.75)
            else:
                self.set_speeds(-MAX_SPEED * 0.75, -MAX_SPEED * 0.35)
            return

        heading_error = (math.atan2(gy - self.y, gx - self.x) - self.theta
                         + math.pi) % (2 * math.pi) - math.pi
        # Steering scaled to cruise speed; large errors → rotate in place so a
        # fast robot doesn't arc wide past the sub-waypoint.
        if abs(heading_error) > 1.2:
            w = MAX_SPEED * 0.4 * (1 if heading_error > 0 else -1)
            self.set_speeds(-w, w)
            return
        turn  = MAX_SPEED * 0.45 * heading_error
        speed = MAX_SPEED * 0.8
        front = self._min_front_range()
        # IR low-obstacle override — but NOT while closing on a victim: a
        # lying victim is itself a low obstacle, and the whole point of the
        # approach is to get within 1.0 m of it.
        if self._repositioning is None and not self._homing:
            ir_f = self._ir_front_m()
            if ir_f < IR_DANGER_M and ir_f + 0.15 < front:
                # Low obstacle beneath the lidar plane (stairs/rubble):
                # remember it, then treat it exactly like a wall dead ahead.
                self._paint_low_obstacle(ir_f)
                front = ir_f
        if front < DANGER_FRONT_M:
            self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
            self._avoid_ticks = 8
        elif front < 0.32 or self._avoid_ticks > 0:
            self._avoid_ticks = max(0, self._avoid_ticks - 1)
            if self._clear_side() == 'left':
                self.set_speeds(0.0, speed * 0.8)
            else:
                self.set_speeds(speed * 0.8, 0.0)
        else:
            self._avoid_ticks = 0
            self.set_speeds(speed - turn, speed + turn)

    # ── Navigation ────────────────────────────────────────────────────────

    def navigate_step(self):
        """
        Follow the itinerary: victim estimates first, then a building-bounded
        sweep. Each tick, in order:

          * retire the current victim waypoint if its victim is already
            confirmed (WP_RETIRE_M), so we do not drive at a solved target;
          * adopt the partner's unvisited estimates once our own list is done
            (PLAN 3.6 assist);
          * steer back inside the building if we have strayed out of it;
          * abandon a waypoint after WAYPOINT_TIMEOUT_S, requeueing a victim
            waypoint once in case the map has since learned the obstruction;
          * escape a local trap by following the obstacle surface when we are
            close to the target but no longer making progress (WALL_FOLLOW);
          * otherwise drive the A* route, scanning on arrival.

        Falls back to straight-line wander only if the whole list is exhausted.
        """
        if self.target_idx >= self._n_video_wps:
            self._maybe_assist_partner()
            # Optional frontier exploration (OFF by default — see
            # EXPLORE_FRONTIERS for the measurement that decided it).
            if EXPLORE_FRONTIERS:
                self._maybe_frontier_target(self.robot.getTime())

        # ── Geofence ── Victims are inside the building; a robot outside it is
        # wasting the mission at best and driving off the world plane at worst
        # (29/07 small run: robot1 ended at (13.4,3.6), beyond the NE wall,
        # stranded off the floor for the last 40 s). Override the itinerary and
        # steer back inside until we are within the envelope again.
        if self._outside_envelope():
            # Head back to the nearest point comfortably inside the envelope
            # (not the bbox edge — that is itself uncertain).
            x0, x1, y0, y1 = self._envelope(slack=2.0)
            gx = min(max(self.x, x0), x1)
            gy = min(max(self.y, y0), y1)
            if not self._geofence_active:
                self._geofence_active = True
                print(f"[{self.robot_id}] Outside building envelope at "
                      f"({self.x:.1f},{self.y:.1f}) — returning to "
                      f"({gx:.1f},{gy:.1f})")
            self._drive_toward(gx, gy, self.robot.getTime())
            return
        self._geofence_active = False

        # Retire video waypoints whose victim is already confirmed. Estimates
        # carry ~1.5 m of error, so a report within 3 m of the estimate IS that
        # victim. 29/07 small run: robot1 found victim4 at (9.02,5.89) then
        # spent 90 s still driving at its estimate (9.56,8.60) — through the NE
        # wall and out of the building.
        while (self.target_idx < min(self._n_video_wps,
                                     len(self._my_waypoints))):
            tx, ty = self._my_waypoints[self.target_idx]
            if self._region_blacklisted(tx, ty):
                print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) "
                      f"skipped — region written off as unreachable")
                self.target_idx    += 1
                self._wp_start_time = None
                self._path          = None
                continue
            if any(math.hypot(tx - rx, ty - ry) < WP_RETIRE_M
                   for rx, ry in self._reported_positions):
                print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) "
                      f"retired — victim already confirmed")
                self.target_idx    += 1
                self._wp_start_time = None
                self._path          = None
            else:
                break

        if self.target_idx < len(self._my_waypoints):
            tx, ty = self._my_waypoints[self.target_idx]

            if self._scanning:
                self._scan_step()
                return

            dx, dy = tx - self.x, ty - self.y
            dist   = math.hypot(dx, dy)

            now_wf = self.robot.getTime()
            if WALL_FOLLOW:
                if now_wf < self._wf_until:
                    if self._wf_should_leave(tx, ty, dist, now_wf):
                        self._wf_until  = 0.0
                        self._path      = None
                        self._wp_best_dist = min(self._wp_best_dist, dist)
                        self._wp_best_t    = now_wf   # re-arm progress clock
                        print(f"[{self.robot_id}] Wall-follow done — "
                              f"{dist:.1f} m from ({tx:.1f},{ty:.1f}), "
                              f"lane open, back to planning")
                    else:
                        self._wall_follow_step()
                        return
                elif (self._wp_start_time is not None
                        and dist < WF_TRIGGER_DIST_M
                        and (now_wf - self._wp_best_t) > WF_NO_PROGRESS_S):
                    self._wf_until   = now_wf + WF_MAX_S
                    self._wf_side    = self._clear_side()
                    self._wf_count  += 1
                    self._wf_hit      = (self.x, self.y)
                    self._wf_hit_dist = dist
                    self._wf_started  = now_wf
                    self._wp_best_t   = now_wf     # re-arm the progress clock
                    if self._wf_count <= 3 or self._wf_count % 5 == 0:
                        print(f"[{self.robot_id}] Trapped {dist:.1f} m from "
                              f"({tx:.1f},{ty:.1f}) — following wall on the "
                              f"{self._wf_side} until a lane opens")
                    self._wall_follow_step()
                    return

            # Waypoint timeout — skip if unreachable within allotted time. Kept
            # short (vs. the ~180 s mission) so one bad video-derived estimate
            # can't eat the budget before the sweep fallback ever starts.
            now = self.robot.getTime()
            if self._wp_start_time is None:
                self._wp_start_time = now
                self._wp_best_dist  = dist
                self._wp_best_t     = now
            # Progress-based abandonment. A flat 40 s wall treats "far away but
            # closing steadily" (medium's waypoints are 20+ m out) the same as
            # "wedged against furniture". Track the best distance achieved:
            # if it hasn't improved in WAYPOINT_STALL_S we are stuck, whatever
            # the clock says — and if it has, keep going up to the hard cap.
            if dist < self._wp_best_dist - 0.3:
                self._wp_best_dist = dist
                self._wp_best_t    = now
            # Only judge a waypoint stalled while we are still FAR from it.
            # Within a few metres the robot is usually threading furniture at
            # centimetre pace — exactly where victims are — and abandoning
            # there costs finds (29/07: robot2 quit both targets by t=90 and
            # never revisited victim3, which it had found at t=131 twice).
            stalled = (STALL_ABANDON
                       and dist > WAYPOINT_STALL_MIN_DIST_M
                       and (now - self._wp_best_t) > WAYPOINT_STALL_S)
            if stalled or now - self._wp_start_time > WAYPOINT_TIMEOUT_S:
                # A timed-out VIDEO waypoint is a likely real victim — requeue
                # it once at the end of the video leg (the map has usually
                # learned the offending wall by then) instead of abandoning it.
                key = (round(tx, 2), round(ty, 2))
                if (self.target_idx < self._n_video_wps
                        and self._wp_retries.get(key, 0) < 1):
                    self._wp_retries[key] = 1
                    self._my_waypoints.insert(self._n_video_wps, (tx, ty))
                    self._n_video_wps += 1
                    print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) timeout — requeued")
                else:
                    # Abandoned for good: remember it and tell the partner, so
                    # neither robot adopts a target that has already eaten a
                    # full timeout (29/07 medium: robot1 adopted two waypoints
                    # robot2 had abandoned, and timed out on them in turn).
                    self._failed_wps.append((tx, ty))
                    self._note_region_failure(tx, ty)
                    self._broadcast_failed(tx, ty)
                    # Also bar it as an exploration target, or the frontier
                    # picker re-selects the very cell we just abandoned
                    # (29/07: robot1 looped on (12.5,-17.1) for 65 s).
                    self._frontier_failed.append((tx, ty))
                    print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) timeout — skipping")
                self.target_idx    += 1
                self._wp_start_time = None
                return

            arrive_r = (ARRIVE_VIDEO_M if self.target_idx < self._n_video_wps
                        else ARRIVE_SWEEP_M)
            stuck_near = (self.target_idx < self._n_video_wps
                          and dist < SCAN_NEAR_M
                          and self._wp_start_time is not None
                          and (self.robot.getTime() - self._wp_best_t)
                              > SCAN_STUCK_S)
            if dist < arrive_r or stuck_near:
                # Arrived. The waypoint puts us NEAR rather than on the victim
                # (see ARRIVE_VIDEO_M) and the camera may be facing anywhere, so
                # scan on the spot rather than trusting the arrival heading.
                self.stop()
                if self._already_reported_nearby(self.x, self.y):
                    self.target_idx    += 1
                    self._wp_start_time = None
                else:
                    self._scanning        = True
                    self._scan_ticks_left = self._scan_duration_ticks
                return

            # ── Plan a route to the goal, follow it as sub-waypoints ──
            # Re-plan when the goal changes, we have no path, or the plan is stale.
            if (self._planned_goal != (tx, ty) or self._path is None
                    or now - self._last_plan_time > NAV_REPLAN_S):
                self._path           = self._plan_path(tx, ty)
                self._path_idx       = 0
                self._planned_goal   = (tx, ty)
                self._last_plan_time = now

            if self._path:
                # Skip sub-waypoints we've already reached
                while (self._path_idx < len(self._path) - 1
                       and math.hypot(self.x - self._path[self._path_idx][0],
                                      self.y - self._path[self._path_idx][1]) < NAV_SUBGOAL_M):
                    self._path_idx += 1
                gx, gy = self._path[self._path_idx]
            else:
                gx, gy = tx, ty     # no route (no map / unreachable) → head straight

            self._drive_toward(gx, gy, now)
        else:
            # All waypoints done — wander with directional bias toward victims:
            # robot1 (north sweep) biased northeast; robot2 (south sweep) southeast.
            front = min(self._min_front_range(), self._ir_front_m())
            if front < 0.25:
                self.set_speeds(-MAX_SPEED * 0.6, -MAX_SPEED * 0.6)
            elif front < 0.30:
                if self._clear_side() == 'left':
                    self.set_speeds(0, MAX_SPEED * 0.7)      # curve left
                else:
                    self.set_speeds(MAX_SPEED * 0.7, 0)      # curve right
            else:
                # Waypoint list exhausted (shouldn't normally happen — the sweep
                # grid alone has hundreds of entries) — hold heading and drift
                # forward rather than stopping dead.
                self.set_speeds(MAX_SPEED * 0.6, MAX_SPEED * 0.6)

    def _region_blacklisted(self, x, y):
        """True if (x, y) lies in an area this robot has repeatedly failed to
        reach. Prevents a robot from sinking its whole mission into one
        unreachable corner (see REGION_BLACKLIST)."""
        if not REGION_BLACKLIST:
            return False
        for (cx, cy, n) in self._fail_regions:
            if n >= REGION_FAIL_N and math.hypot(x - cx, y - cy) <= REGION_FAIL_R:
                return True
        return False

    def _note_region_failure(self, x, y):
        """Record a failure to reach (x, y), clustering nearby failures."""
        for i, (cx, cy, n) in enumerate(self._fail_regions):
            if math.hypot(x - cx, y - cy) <= REGION_FAIL_R:
                self._fail_regions[i] = (cx, cy, n + 1)
                if n + 1 == REGION_FAIL_N:
                    print(f"[{self.robot_id}] Region ({cx:.1f},{cy:.1f}) "
                          f"written off after {n + 1} failures")
                return
        self._fail_regions.append((x, y, 1))

    def _maybe_assist_partner(self):
        """PLAN §3.6 partner assist: this robot's own video leg is finished —
        adopt the partner's still-unconfirmed victim estimates (nearest-first)
        ahead of the sweep leg. Late in a mission both robots may briefly
        chase the same estimate; the near-prior-report gate prevents double
        reports, and finding ratio (40%) outweighs the wasted metres (15%).
        Re-evaluated every 15 s while the video leg stays exhausted, so a
        waypoint skipped because the partner was working it is reconsidered
        once the partner abandons it. Previously adopted waypoints join this
        robot's own list, so they are never adopted twice."""
        if not ASSIST_ENABLED:
            return
        now = self.robot.getTime()
        if now < self._assist_next_t:
            return
        self._assist_next_t = now + 15.0
        own = {(round(x, 2), round(y, 2))
               for x, y in self._my_waypoints[:self._n_video_wps]}
        cands = []
        for vx, vy in self._video_targets:
            if (round(vx, 2), round(vy, 2)) in own:
                continue
            # 1.2 m: an estimate this close to a confirmed report (ours or the
            # partner's, via channel 73) is that victim — already handled.
            if any(math.hypot(vx - rx, vy - ry) < 1.2
                   for rx, ry in self._reported_positions):
                continue
            # Never adopt a target either robot has already abandoned: it cost
            # a full timeout once and will cost another. Exploring unknown
            # space is a better use of the remaining seconds.
            _failed = (self._failed_wps if ASSIST_SKIP_OWN_ONLY
                       else self._failed_wps + self._partner_failed)
            if ASSIST_SKIP_FAILED and any(math.hypot(vx - fx, vy - fy) < 1.5
                                          for fx, fy in _failed):
                continue
            if self._region_blacklisted(vx, vy):
                continue
            # Skip the waypoint the partner is actively working right now
            # (broadcast on channel 73) — adopting it just duplicates effort.
            if (self._partner_target is not None
                    and math.hypot(vx - self._partner_target[0],
                                   vy - self._partner_target[1]) < 0.6):
                continue
            cands.append((vx, vy))
        if not cands:
            return
        cands = self._nn_order(cands, (self.x, self.y))
        # Insert at the CURRENT queue position (we may be mid-sweep by now)
        # and extend the video leg to cover the adopted waypoints so the
        # requeue-once and repositioning logic treat them as victim targets.
        for i, wp in enumerate(cands):
            self._my_waypoints.insert(self.target_idx + i, wp)
        self._n_video_wps = max(self._n_video_wps,
                                self.target_idx + len(cands))
        self._wp_start_time = None
        self._path = None
        print(f"[{self.robot_id}] Partner assist: adopted "
              + ", ".join(f"({x:.1f},{y:.1f})" for x, y in cands))

    def _scan_step(self):
        """
        Rotate in place at an arrived waypoint, checking the camera each tick,
        for up to SCAN_DURATION_S before giving up and moving on. Fires
        report_victim() and ends the scan as soon as the gate passes.
        """
        conf, depth, _ = self._camera_victim_confidence()
        # Spotted a candidate while scanning → hand off to homing (which drives
        # to <1 m before reporting) rather than reporting from arm's length.
        if conf >= APPROACH_ENTER_CONF and not self._victim_false_positive_zone():
            self._scanning        = False
            self._homing          = True
            self._home_start_time = self.robot.getTime()
            self._home_lost_ticks = 0
            self._home_best_depth = float('inf')
            self._home_best_t     = self.robot.getTime()
            return

        self._scan_ticks_left -= 1
        if self._scan_ticks_left <= 0:
            self._scanning      = False
            self.target_idx    += 1
            self._wp_start_time = None
            self.stop()
            return

        self.set_speeds(-SCAN_TURN_SPEED, SCAN_TURN_SPEED)

    # ── Main loop ─────────────────────────────────────────────────────────

    def run(self):
        print(f"[{self.robot_id}] Mission started")
        _pos_tick = 0
        while self.robot.step(self.timestep) != -1:
            self.update_odometry()
            _k = min(1.0, (self.timestep / 1000.0) / TILT_EMA_TAU_S)
            self._tilt_deg_now += _k * (self._read_tilt() - self._tilt_deg_now)
            self._read_squad_messages()
            self._update_map_from_lidar()
            self._learned_map_step()
            self._vision_telemetry()

            # ── Tilt guard ── (before any driving state can act)
            now_g = self.robot.getTime()
            if self._tilt_deg_now > TILT_RECOVER_HI:
                # Fully inverted: wheels-up, nothing to be done. Freeze —
                # learned-map merge duties above this line keep running.
                self.stop()
                continue
            if self._tilt_deg_now > TILT_RECOVER_LO:
                # On our side / high-centred (forklift tongs, rubble): rock
                # with straight AND turning pulses to slide off, in endless
                # wiggle/rest cycles — physics jitter sometimes frees what a
                # single burst cannot.
                if self._recover_start is None:
                    self._recover_start = now_g
                    print(f"[{self.robot_id}] Tilt {self._tilt_deg_now:.0f}° "
                          f"— attempting recovery wiggle")
                cycle = (now_g - self._recover_start) % (RECOVER_MAX_S
                                                         + RECOVER_REST_S)
                if cycle < RECOVER_MAX_S:
                    phase = int(cycle / RECOVER_PULSE_S) % 4
                    vl, vr = ((0.45, 0.45), (-0.45, -0.45),
                              (0.5, -0.2), (-0.2, 0.5))[phase]
                    self.set_speeds(MAX_SPEED * vl, MAX_SPEED * vr)
                else:
                    self.stop()                    # rest, then retry
                continue
            self._recover_start = None
            if (self._tilt_deg_now > TILT_ABORT_DEG
                    and now_g > STARTUP_SPIN_S + 1.0):
                if self._tilt_hi_since is None:
                    self._tilt_hi_since = now_g
                if now_g - self._tilt_hi_since >= TILT_SUSTAIN_S:
                    if now_g >= self._tilt_escape_until:
                        print(f"[{self.robot_id}] Tilt "
                              f"{self._tilt_deg_now:.0f}° — backing off")
                    self._tilt_escape_until = max(self._tilt_escape_until,
                                                  now_g + TILT_ESCAPE_S)
                    self._paint_low_obstacle(0.10)   # whatever we're climbing
            else:
                self._tilt_hi_since = None
            if now_g < self._tilt_escape_until:
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                continue

            # Startup 360° spin: let the lidar paint the foyer's REAL walls
            # before the first route commits (rotate-in-place ≈2.5 rad/s →
            # a full turn comfortably inside STARTUP_SPIN_S). The first A*
            # plan then already knows where the foyer's exits are.
            now_t = self.robot.getTime()
            if now_t < STARTUP_SPIN_S:
                w = MAX_SPEED * 0.4
                self.set_speeds(-w, w)
                continue
            if not self._spin_done:
                self._spin_done = True
                self.stop()
                self._path = None          # plan the first route on the spun map
                print(f"[{self.robot_id}] Startup spin complete — "
                      f"{int(self._nav_wall.sum()) if self._nav_wall is not None else 0} "
                      f"wall cells learned")

            # Broadcast our position to partner once per second
            _pos_tick += 1
            if _pos_tick >= int(1000 / self.timestep):
                _pos_tick = 0
                self._broadcast_position()

            # Staggered start: robot2 holds still until robot1 clears doorway A
            if self._startup_ticks > 0:
                self._startup_ticks -= 1
                self.stop()
                continue

            # Homing takes priority: if we're closing on a victim, keep at it;
            # otherwise look for one to start homing on, else follow the route.
            if not self._homing:
                self._maybe_start_homing()
            if self._homing:
                self._approach_victim()
            else:
                self.navigate_step()


if __name__ == "__main__":
    SARRobot(_robot).run()