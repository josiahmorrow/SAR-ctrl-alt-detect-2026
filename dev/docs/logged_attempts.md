# Logged Simulation Attempts

Record of every competition run: world, score, observations, and what changed.

---

## Attempt 1 — 2026-06-25

| Field | Value |
|---|---|
| World | `worlds/small_world.wbt` |
| Score | **0.010 / 1.000** |
| Video analysed | `recordings/small_world_flyover.mp4` |
| Commit / code state | Session 3 (camera undistortion added, preflight rewrite) |

### Score breakdown (estimated)
| Component | Weight | Estimated score |
|---|---|---|
| Victim Finding | 40% | ~0 (no robots confirmed any victim) |
| Efficiency | 15% | ~0 (exhausted waypoints quickly, wandered) |
| Coordination | 15% | ~0 (no confirmed victims to coordinate) |
| CSV — victim positions | 15% | **0** (CSV was 0 bytes — zero detections) |
| Map — wall accuracy | 15% | ~0.01 (map generated but inaccurate) |

### Outputs generated

**`sim_logs/victim_location_estimates.csv`** — 0 bytes (empty)  
No victims were found by the video analysis. All candidate blobs were either
below `MIN_CLUSTER_VOTES=3` or outside the HSV detection window.

**`sim_logs/map_estimate.png`** — 12 743 bytes  
Map was generated but only achieved a tiny match score. Reference format below:

> ![Example map format](assets/example_map.png)  
> *Left: Webots world top-down. Right: expected 600×600 binary output (walls black, free white).*

### Root causes identified

1. **Empty CSV — `MIN_CLUSTER_VOTES = 3` too strict**: the drone overflight is fast.
   A victim visible for only 1–2 sampled seconds (at 1 fps) never reaches 3 votes
   and is discarded even if the detection is correct.

2. **Empty CSV — HSV thresholds too narrow**: the detection window
   (`S ≥ 80, V ≥ 60`) excluded dark-hair / dark-clothing victims visible from
   above (typical `V < 60`). The drone top-down view also has no robot chassis in
   frame, so the red exclusion was unnecessarily narrowing the range.

3. **Empty CSV — `ORIGIN_EXCL_M = 3.0 m` too large**: the exclusion zone was
   set to 3 m, yet victim 1 is only ~4.6 m from origin. A calibration error of
   ≥ 1.5 m would push it inside the exclusion zone.

4. **Zero victim findings — depth gate always failing**: `_check_camera_victim()`
   required `depth <= REPORT_RANGE_M (2.0 m)`. The Webots depth camera returns
   `inf` for pixels that are out-of-range or poorly textured. Since `inf > 2.0`,
   the streak **never built up** even when the robot was centred on a victim.

5. **Zero victim findings — arrival 1.0 m from waypoint**: robots stopped when
   within 1.0 m of the waypoint. Waypoints are ~0.5 m from victims → robots were
   as far as 1.5 m from the actual victim centroid, outside the 1.0 m scoring
   threshold.

6. **Waypoint timeout 25 s too short**: at ≤ 0.25 m/s, a waypoint 12 m away
   needs ~48 s to reach. Waypoints were being skipped before arrival.

---

## Attempt 2 — 2026-06-25

| Field | Value |
|---|---|
| World | `worlds/small_world.wbt` |
| Score | **0.010 / 1.000** |
| Note | Same as Attempt 1 — `.last_recording` cache used stale outputs |

Identical result because the `.last_recording` cache file told `run_preflight()`
the video had already been analysed, so it reused the empty CSV.

---

## Changes made after Attempt 2 (next run: Attempt 3)

### `prepare_mission.py` — video extraction

| Parameter | Before | After | Reason |
|---|---|---|---|
| `MIN_CLUSTER_VOTES` | 3 | **2** | Brief overflight gives 1–2 frames; 3 was too strict |
| `HUMAN_AREA_MIN_F` | 0.08 | **0.015** | Victims appear very small at cruising altitude |
| `HUMAN_AREA_MAX_F` | 12.0 | **14.0** | Wider margin for low-altitude larger silhouettes |
| `CIRCULARITY_MIN` | 0.25 | **0.10** | Victims from above are non-circular (elongated silhouette) |
| `ORIGIN_EXCL_M` | 3.0 | **2.0** | Tighter exclusion around 1×1 m mat |
| HSV skin range | `S≥20, V≥60` | **`S≥18, V≥50`** | Catch slightly desaturated / dimly lit skin |
| HSV clothing range | `H 10–165, S≥80` | **`H 0–180, S≥45, V≥35`** | All colours, lower saturation floor |
| HSV dark range | not present | **`S≥60, V 10–55`** | Catch dark hair / dark clothing from above |
| Camera intrinsic calibration | not present | **`calibrate_intrinsics()`** | Chessboard + ArUco fallback |

### `proposed_solution.py` — robot navigation & detection

| Parameter | Before | After | Reason |
|---|---|---|---|
| `CAM_CONFIRM` | 0.35 | **0.20** | Lower gate; depth gate still guards against far FPs |
| `CAM_STREAK_N` | 3 | **2** | Faster confirmation (2 consecutive frames) |
| `REPORT_RANGE_M` | 2.0 | **3.5** | Depth camera must see victim ≤ 3.5 m |
| Arrival threshold | 1.0 m | **0.55 m** | Robot gets close enough to score (waypoint ≤ 0.5 m from victim → total ≤ 1.05 m) |
| Arrival confidence | `≥ CAM_CONFIRM` | **≥ 0.15** | Relaxed at arrival — we navigated here deliberately |
| Depth gate (streak) | `depth ≤ 2.0 m only` | **`in_range OR (no_depth AND conf ≥ 0.5)`** | Depth camera returns inf for valid victim pixels |
| Waypoint timeout | 25 s | **50 s** | Farthest waypoint ~12 m; robot needs ~48 s |

### Cache cleared
`.last_recording` deleted — next run will re-analyse the flyover with updated parameters.

---

## Victim ground truth (small_world) for reference

| Victim | Webots (x, y) | ArUco frame (x, y) | Nearest sweep waypoint | Distance |
|---|---|---|---|---|
| victim1 | (−8.11, −10.54) | **(3.39, −3.04)** | Robot 2: (3.0, −3.5) | 0.55 m |
| victim2 | (−6.21, −11.44) | **(5.29, −3.94)** | Robot 2: (5.5, −3.5) | 0.49 m |
| victim3 | (−8.68, −1.99)  | **(2.82, +5.51)**  | Robot 1: (2.5, +5.71) | 0.36 m |
| victim4 | (−3.03, −0.96)  | **(8.47, +6.54)**  | Robot 1: (8.5, +6.54) | 0.03 m |

*ArUco frame offset: OriginMarker parent at Webots (−11.5, −7.5). All sweep waypoints are within 0.55 m of actual victims — scoring threshold is 1.0 m.*

---

*Add future attempts below this line.*

---

## 30/07 evening — what the top-down screenshots explained, and two ideas they killed

Joe captured top-down views of all three worlds. They answered a question no log
could: **the robots do not start inside the building they must search.**

* small_world — the takeoff mat sits on the paving *south of the house*, both
  robots beside it. Reaching any victim means passing the front door at the
  bottom of the central hallway.
* medium_world — the mat is in the truck bay, a separate hall from the searched
  building, next to the semi-trailer.
* large_world — the mat is on the forklift apron.

This re-reads the "mobility ceiling" from the logs. small_world's robot2
oscillated at mat y = −6.0 while its two victims sat at world y = −10.7 and
−11.4: it was pressed against the **outside** of the south wall, 2–3 m from
victims on the other side of it, with the doorway a few metres further west. It
was never wedged in furniture — it was outside the building, re-planning straight
through a wall.

### Idea 1 — Bug2 with a geometric leave test — KEPT (neutral, better founded)

A follow that stops after 6 s covers ~2 m at 0.33 m/s, i.e. reliably just short
of the door. Replaced the stopwatch with a proper Bug2 leave test: keep following
until the robot is measurably closer than at the hit point **and** the lidar
shows a lane wider than the remaining distance in the goal's bearing (that is
what a doorway looks like from the robot), or the follow returns to its own hit
point (the surface encloses us).

| Build | small | medium |
| --- | --- | --- |
| baseline (timed 6 s) | 0.551 / 0.561 | 0.473 / 0.478 |
| leave test | 0.549 | 0.473 |

Neutral on both. Kept because it is principled rather than tuned, and the judging
worlds are unseen — but it did **not** break the ceiling, and honesty demands
saying so.

### Idea 2 — crediting the waypoint deadline with follow time — REJECTED (−0.343)

Shipped alongside the leave test, on the reasoning that time spent walking to a
door is progress and should not count against the waypoint's 40 s deadline.

| Build | small run 1 | small run 2 |
| --- | --- | --- |
| leave test + deadline credit | **0.208** (0/4, one false report) | **0.208** |
| leave test alone | 0.549 (1/4) | — |

The deadline is not overhead to be reclaimed: it is what pushes a robot off a
hopeless target and onto the sweep leg — and the sweep leg is where
small_world's only find actually happens. A/B isolated the credit as the entire
loss. Do not reintroduce a deadline extension of any kind without re-measuring.

### Idea 3 — registering the flyover map onto the startup spin — REJECTED offline

The natural follow-up: don't discard a map that scores 0.843, *align* it. The
spin's lidar walls are registered exactly (the mat defines the frame), so
cross-correlating should recover the offset and let A* route through real
doorways. Tested offline in `tools/align_check.py` — no Webots run needed:

| World | spin wall cells | best fit | flyover-vs-truth, 0.3 m band |
| --- | --- | --- | --- |
| small | 141 | rejected | 0.258 → 0.258 |
| medium | 65 | rejected | 0.062 → 0.062 |
| large | 44 | +0.7, −1.8 m | 0.104 → **0.025** (worse) |

Dead for two independent reasons. There is nothing to register against — from an
open mat the spin sees 44–141 wall cells — and the flyover map is not merely
offset: only 6–26 % of its wall cells fall within 0.3 m of a real wall, so no
translation can fix it. On large the best-scoring shift made agreement four times
worse. The supervisor's 0.843 is an IoU with its own best-shift search and 0.2 m
thick walls, generous about exactly the errors that ruin planning.

**Consequence:** the lidar-first empty routing grid is now an evidence-based
decision rather than a reaction to one bad run, and "align the flyover map" can
be struck off the list for good.

### Idea 4 — shorter waypoint deadline — REJECTED (−0.439)

If lengthening the deadline cost 0.343 because it delayed the sweep leg, then
shortening it should gain for the same reason. It does the opposite:

| `WAYPOINT_TIMEOUT_S` | small |
| --- | --- |
| 25 s | **0.116** (0/4, no reports at all) |
| 40 s (shipping) | 0.549–0.555 (1/4) |
| 40 s + up to 20 s credit | 0.208 (0/4) |

At 25 s the robot abandons a victim waypoint before it can arrive; at 60 s it
never reaches the sweep. **40 s sits on a knife edge** — 0.343 worse if
lengthened, 0.439 worse if shortened. That is worth knowing in itself: the build
is tuned to a narrow local optimum, which is a fair thing to be wary about on
unseen judging worlds.

### Idea 5 — structure filter on the learned map — REJECTED (−0.013)

The map deliverable's score moves *against* coverage: 45 % → 0.892, 49 % → 0.808,
53 % → 0.828, 64 % → **0.240**. Ground truth contains walls only, and lidar cannot
tell a wall from a sofa, so every room covered adds furniture as false positives.

`tools/map_window_check.py` rebuilds the deliverable offline from the saved
learned grids. It reproduced the official 0.240 exactly and ruled out the framing:
the crop is already centred to within 0.1 m, and percentile / max-mass /
largest-component windows are all worse. Keeping only cells belonging to a
straight run ≥ 1.5 m (walls are, furniture is not) lifted that grid 0.2431 → 0.2738.

In the regime we actually run in, it does nothing:

| Build | small map | small overall | medium |
| --- | --- | --- | --- |
| unfiltered | 0.808 | 0.549 | 0.457 / 0.473 |
| straight-run filter | 0.795 | 0.547 | 0.457 / 0.473 |

Reverted. The offline gain exists only at a coverage we do not normally reach, and
paying a certain 0.013 for a hypothetical is the same mistake as every other
unmeasured "improvement" in this list. **The cliff itself stays on the record as a
known fragility:** if a judging world lets the robots roam further than ours do,
the map deliverable can collapse from ~0.85 to ~0.24, costing ~0.10 overall.

### 31/07 morning — the off-map excursions, diagnosed and (mostly) understood

Joe's screenshot showed a robot 8-10 m south of small_world's house, past the
outdoor sofa. A DEBUG_VISION telemetry run (official score 0.483 — behaviour
representative) settled which of six candidate mechanisms is real:

- **Camera homing: exonerated.** conf = 0.00 on all 264 outside ticks.
- **Outside sweep waypoints: exonerated.** Every target held while outside was an
  INTERIOR video waypoint.
- **Odometry: not implicated** at the moment of reports (a TRUE report requires
  believed ≈ true within 1 m).
- **Confirmed mechanism:** the planner. Open ground costs 1, walls cost 30; with
  the doorway unknown, orbiting the building is genuinely the cheapest route to
  an interior goal — and wall-follow's clear-side rule steers AWAY from an
  exterior wall. robot2 spent 240 of ~700 ticks >1 m outside the wall bbox
  (deepest 3.19 m); the geofence never fired once at slack 3.5.

**Fence tightening (slack 3.5 → 2.0): measured NEUTRAL.** 0.483 with identical
finding score; the fence fired zero times even at 2.0 (that run's excursions
stayed shallower). Not merged — 3.5 keeps the margin against the herding failure
of 29/07 and the tighter value demonstrated no benefit.

**Left for Phase 2, deliberately:** the real fix is teaching wall-follow that
outside the envelope the BUILDING side is the side to follow (its current rule
is exactly backwards there), and/or an entry-first behaviour: reach the interior
before chasing any victim waypoint. Both are more than a deadline-day change.
The excursions cost efficiency (F everywhere), not victims, so the expected
value today was always modest.

### Also observed: large_world's tail is worse than documented

One large run in this batch scored **0.066** (0/5, seven tilt events, 11 %
coverage) with a build measured at 0.392/0.392/0.394 immediately before it. The
documented spread was 0.388–0.471 over six runs; the real tail includes runs where
both robots spend the mission fighting obstacles. Any single large_world number —
ours or anyone else's — should be treated as almost uninformative.
