# Proposal: replace the navigation map and planner core (NOT STARTED - needs Joe's approval)

Status: draft, 25/11/2026. The brief requires approval before replacing the
planner or rewriting more than ~300 lines.

## Evidence that the layer is structurally wrong

1. **The map is binary, with no memory of how often a cell was seen free.**
   One lidar endpoint makes a cell a wall; only a ray passing within 3 m can
   clear it. Anything that moves through the robot's view becomes a permanent
   wall. Measured: robot1 crossing small's front door sealed it in robot2's map
   for the whole mission (exp01/base600: 600 s outside). exp02 masks the
   partner robot, a patch for one moving object: victims' bodies, doors and
   robots seen from a tilted chassis still paint permanent walls.
2. **Walls are a finite cost (30 per cell).** A route through a wall beats any
   doorway more than ~3 m away, so the robot drives at walls and relies on the
   reactive layer plus a 40 s timeout to give up. Making walls hard (exp01)
   cannot work on top of (1): noise seals real doorways.
3. **Low obstacles are invisible to the planner except as 3x3 patches dropped
   when the IR or tilt guard fires.** Large: both robots beached within
   ~10-30 s in every run (0/5, 18 m travelled in 600 s).
4. **Behaviour is a stack of overriding heuristics** (geofence, wall-follow
   with leave test, scan-when-stuck, timeouts with requeue, partner assist,
   reposition) whose interactions decide outcomes: exp03 and exp04 each fixed
   a measured fault and lost a victim elsewhere.

## Proposed replacement (behind the existing interfaces)

Same inputs (lidar, IR, accelerometer, odometry, partner broadcasts), same
outputs (`_plan_path` returning sub-waypoints, `_nav_wall` for the learned-map
deliverable). Roughly 250-350 lines replacing ~200.

* **Log-odds occupancy grid** (0.10 m): every beam ray-cast; hit +0.9,
  pass -0.4, clamped to [-4, 4]; occupied when > 1.0. A moving object is
  cleared by the next few passes through its cell, so doorways reopen by
  themselves. Partner mask kept.
* **Planner on hard, inflated obstacles**: occupied cells blocked, inflated by
  the chassis radius (~0.18 m); unknown cells traversable (optimistic), with
  a clearance cost ring. Replan when the current path crosses a newly
  occupied cell, not on a timer. A route around a wall to its door then
  falls out of the search instead of a wall-follow heuristic.
* **Low obstacles as first-class map content**: IR returns below the lidar
  plane and tilt events write occupied, inflated cells with the same log-odds
  update, so a beaching spot is routed around next time.
* **Retire the heuristics the planner makes redundant**, one at a time and
  measured: wall-follow, geofence, scan-when-stuck.

## How it would be judged

Seeds 0/1/2 on small and medium at the current rung, large x1 (deterministic
here); keep only if the mean find count rises on both and false reports do
not. Offline first: a replay test that feeds recorded lidar scans from the
`diag` runs into the new grid and checks the front door of small stays open.
