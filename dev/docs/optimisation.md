# Optimisation — Improving the Video Information Extraction Score (30%)

> **Scope:** the 30% "Video Information Extraction" component only. It is the average of
> two equally-weighted halves:
> - **Victim location estimate accuracy** (15%) → `sim_logs/victim_location_estimates.csv`
> - **Map estimate accuracy** (15%) → `sim_logs/map_estimate.png`
>
> Owner file to improve: `controllers/proposed_solution/prepare_mission.py`.
> Read-only reference for how we're graded: `controllers/sar_marking_supervisor/sar_marking_supervisor.py`.

---

## 1. How we are actually scored (from the supervisor code)

`_generate_ground_truth_map()` in `sar_marking_supervisor.py` is the exact ground truth our
`map_estimate.png` is compared against. The mechanics that matter:

- **Sources:** ground truth is built from `Wall` + `Window` + `Door` nodes.
- **Ceiling/roof filter:** any node at **`z > 1.0` is skipped** (e.g. the elevated `size 10 13`
  slab at `z 2.4`). Only ground-level structure is drawn.
- **Geometry:** each wall → 4 corners via `position + rotation_matrix · (±size/2)`, drawn as a
  **filled black polygon** on a white canvas.
- **Centering (critical):** the map is centered on the **bounding-box centroid of ALL wall
  corners** (`center_x = (all_min_x + all_max_x)/2`, likewise Y) — **NOT** on the origin marker.
- **Canvas:** 600×600 px, **0.05 m/px** (30×30 m), **+X → right, +Y → up** (Y flipped in image
  space: `img_y = (world_max_y - wy)/resolution`).
- **Victim ground truth:** the `VICTIM_MARKER` node position inside the Victim PROTO (the marker,
  **not** the body centroid). Proximity radius for the finding task is 1.0 m.
- **Our CSV frame:** the supervisor adds `coordinate_offset` (= `OriginMarker.getPosition()`) back
  onto our estimates to convert them to Webots world coords. So our CSV must stay in the
  **ArUco/origin frame** (as it currently is).
- **Count consistency is heavily weighted:** the example marking report scored **0.667 despite
  0.00 m position error**, purely because it submitted 3 estimates when truth was 2.

### 1.1 The probable `0.078`-killer (highest-value single fix)

Our current `build_wall_map()` / `make_map_to_src_H()` places the **ArUco origin (0,0) at pixel
(300,300)**:

```python
M_map_world = np.array([
    [ MAP_SCALE,  0.0,       -HALF_M],
    [ 0.0,       -MAP_SCALE,  HALF_M],
    [ 0.0,        0.0,        1.0   ]])
return np.linalg.inv(H_pix_world) @ M_map_world
```

But the ground truth places the **wall bounding-box center** at (300,300). The origin marker sits
at the edge of the scene, not the middle, so **our entire wall map is translated by several metres
relative to GT → near-zero pixel overlap regardless of reconstruction quality.** Fixing centering
alone should move the map score substantially.

### 1.2 The biggest enabler

We have the exact GT-generation code **and** the `.wbt` files for all three dev worlds
(small/medium/large). We can therefore generate **pixel-exact ground truth offline** and score our
outputs in seconds, without launching Webots. Every phase below rides on building that harness first.

---

## 2. Output contract (do not break what already passes format)

- **CSV:** no header, `x,y` per line, numeric only, **one row per real victim** (over-reporting is
  penalised). Dedup + count-calibrate.
- **PNG:** exactly **600×600**, **0.05 m/px**, **centred on the wall bounding box**, walls pure
  `[0,0,0]`, free space pure `[255,255,255]`, **no greys** (no JPEG/anti-alias artifacts), walls at
  **0.2 m ≈ 4 px** thickness.

---

## 3. The plan (phased, prioritised)

### Phase 0 — Offline evaluation harness (do this first; everything depends on it)

1. **Extract the GT generator into a reusable offline module.** Lift `_generate_ground_truth_map` +
   `_get_wall_bounding_box` logic into a script that parses a `.wbt` directly (or drives a headless
   Supervisor once) to emit `ground_truth_map_<world>.png` for small/medium/large. Reuse the *exact*
   code so the format matches the server byte-for-byte.
2. **Extract GT victim positions** the same way (Victim nodes → `VICTIM_MARKER` local offset from
   `Victim.proto`, transformed into the ArUco frame). One supervisor run also prints them if parsing
   the proto is fiddly.
3. **Write a scorer** that loads our `map_estimate.png` + a GT map and reports **IoU on the wall
   class, F1, and pixel accuracy**, plus a victim-CSV scorer (matched-distance error + count
   penalty). We don't know the server's exact formula, so track all three and optimise IoU/F1 (the
   discriminating ones).
4. **Run the current pipeline on all three worlds and _look_ at the outputs** side-by-side with GT
   (per `BERTUS.md`). This resolves an open question the `.wbt` alone can't settle: whether GT walls
   render as thin 0.2 m lines or large filled blocks (several `Wall` nodes have multi-metre `size`
   values). **Tune to whatever the rendered GT actually is — don't guess.**

**Impact:** turns a 3-minute opaque sim loop into a sub-second local metric. This is what makes the
rest tunable.

### Phase 1 — Fix map centering & format (quick, high ROI)

5. **Re-center the estimate on the reconstructed walls' bounding box**, not the ArUco origin —
   replicate the GT's `center = (min+max)/2` convention so the wall structure lands at pixel
   (300,300).
6. **Verify axis/orientation** matches GT (+X right, +Y up with Y-flip). Confirm the ArUco frame is
   rotation-aligned with the Webots frame (it was a pure +11.5 / +7.5 translation for `small_world`
   per `logged_attempts.md`; check medium/large hold too).
7. **Enforce strict format:** pure `[0,0,0]`/`[255,255,255]` (no greys), exactly 600×600, walls
   dilated to 0.2 m ≈ 4 px thickness, exterior walls closed.

**Impact:** likely the single largest map-score jump, independent of reconstruction quality.

### Phase 2 — Fix georeferencing / drone pose (root cause of *both* halves)

The map and victim positions are both bounded by how accurately each video frame maps to world
coordinates. Both current methods drift:

8. **Stop double-integrating the accelerometer for position** — it drifts quadratically over a
   60–90 s flight (almost certainly the documented "Y-drift"). Use the IMU for **heading
   (gyro+compass) and altitude prior only**.
9. **Replace median-translation ORB with robust full-transform estimation**
   (`estimateAffinePartial2D` / `findHomography` + RANSAC) so yaw and altitude changes are captured,
   not just translation.
10. **Register each frame to the accumulating global mosaic, not the previous frame** (map-to-frame
    instead of frame-to-frame). This stops linear drift accumulation — errors correct against a fixed
    reference instead of compounding.
11. **A/B the pose sources in the harness** (IMU-only vs vision-only vs fused) and pick empirically —
    the current `use_imu = bool(drone_traj)` preference may be actively worse than vision.

**Impact:** unlocks the ceiling for both map IoU (thin lines need ~1–2 px / 5–10 cm accuracy) and
victim position accuracy.

### Phase 3 — Rebuild wall segmentation to match GT

12. **Drop global Otsu "dark = wall".** Once the mosaic is sharp (Phase 2) and we can see the real GT
    (Phase 0), segment walls by their actual appearance/structure: sample wall colour/texture from
    the mosaic, and/or detect long axis-aligned linear structures, snap collinear segments, and
    rasterize at 0.2 m thickness.
13. **Reject furniture** (the README requires walls only) via length/aspect-ratio/axis-alignment
    filters — furniture is compact, walls are long thin room boundaries.
14. **Handle unvisited regions** deliberately (the drone doesn't cover everything) — decide free vs
    unknown to maximise IoU against GT.

### Phase 4 — Victim estimates (accuracy + count consistency)

The example report scored 0.667 despite **0.00 m error**, purely from a 3-vs-2 **count** mismatch —
count consistency is heavily weighted.

15. **Use temporal persistence as the count signal:** with pose fixed, a real victim appears at a
    *stable world coordinate across many frames*; false positives (carpet, furniture) are transient
    or jump around. Cluster by world position + require persistence.
16. **Calibrate the count threshold** on the three known worlds (known victim counts) rather than the
    arbitrary `MAX_VICTIM_ESTIMATES=4` / `MIN_CLUSTER_VOTES=1`. Bias toward precision — over-reporting
    is penalised.
17. **Optionally build an appearance model** from the known victim textures
    (`protos/textures/*_skin/hair/suit`) if HSV + persistence still over/under-counts.
18. **Report against `VICTIM_MARKER` positions** (what the server uses), not body centroids.

### Phase 5 — Validation, server calibration, and format/caching lock-in

19. **Run all three worlds through the harness**, confirm map IoU and victim error/count improve
    together.
20. **Calibrate to the real server once:** submit a run, read the returned `Map Estimate Accuracy
    Score` / `Victim Location Estimate Score`, and check which local metric best predicts the server —
    then optimise that one.
21. **Fix the stale-cache trap** that cost Attempts 1 & 2 identical `0.010` scores (`.last_recording`
    reused empty outputs) — verify `outputs_are_fresh()` invalidates correctly on **code** changes,
    not just recording changes.
22. **Confirm fresh-clone reproducibility** and exact submission format.

---

## 4. Sequencing

- **Fastest wins:** Phase 0 (harness) → Phase 1 (centering). Low-effort, likely recovers most of the
  lost map score on their own.
- **Deepest lever:** Phase 2 (pose) — it raises the ceiling for everything downstream, so do it before
  heavy tuning in Phases 3–4.
- Everything is measurable offline after Phase 0, so we iterate fast and only touch the real server
  for final calibration (Phase 5).

## 5. Two things to confirm empirically in Phase 0 (before committing to Phase 3)

1. **Are GT walls thin 0.2 m lines or large filled blocks?** Several `Wall` nodes have multi-metre
   `size` values; the supervisor fills polygons of `size[0] × size[1]`. Render the real GT and look.
2. **Is the ArUco frame truly rotation-aligned with the Webots frame in all three worlds?** It was a
   pure translation (+11.5, +7.5) for `small_world`; verify medium/large.

---

## 6. Reference data (small_world)

**Victim ground truth** (from `logged_attempts.md`):

| Victim  | Webots (x, y)   | ArUco frame (x, y) |
|---------|-----------------|--------------------|
| victim1 | (−8.11, −10.54) | **(3.39, −3.04)**  |
| victim2 | (−6.21, −11.44) | **(5.29, −3.94)**  |
| victim3 | (−8.68, −1.99)  | **(2.82, +5.51)**  |
| victim4 | (−3.03, −0.96)  | **(8.47, +6.54)**  |

**Frame offset:** `OriginMarker` parent at Webots (−11.5, −7.5) ⇒ `ArUco = Webots + (11.5, 7.5)`.

**Map format:** 600×600 px, 0.05 m/px (30×30 m), black `[0,0,0]` = wall, white `[255,255,255]` =
free, no greys, walls 0.2 m (4 px) thick, **centred on the wall bounding box**.

---

## 7. Session log — 2026-07-12 (Phase 3 rebuild + Phase 2 refinement)

### What was established (measured, offline, via the harness + `accum.npz`)

1. **Per-pixel brightness cannot segment walls.** GT-wall vs free cells have
   near-identical min/mean brightness distributions (57.2 vs 58.1 mean); the best
   possible threshold on any per-pixel signal reaches IoU 0.08 *with perfect
   alignment*. The old "walls are bright in the min composite" method (69 px,
   IoU 0.000) was unfixable by tuning.
2. **The usable wall signal is parallax smear**: `spread = mean − min` composite.
   3 m walls are the strongest view-dependent structure; they appear as thick
   bright bands centred near the wall line. Long axis-aligned walls integrate
   into strong row/column projection peaks even where locally noisy.
3. **The mosaic is misregistered in the ArUco frame**: globally ~(−1.2, −0.9) m,
   but per-wall offsets scatter ±1.8 m (std 1.2 m) — a local, non-rigid warp. A
   global similarity fit explains only a small part of it.
4. **Victim estimates inherit exactly this warp.** Raw detections cluster within
   ±5 cm per victim (the detector is fine) but each dive's pose carries its own
   1–2 m bias. Fixing cluster statistics (mean/median/weighting) cannot help;
   only pose accuracy can. Root causes: (a) all absolute anchors come from the
   0.75 m-wide origin mat at one corner of the scene — angular/scale error
   extrapolates to metres at the far side; (b) similarity-only pose model;
   (c) dive samples have the worst chains.

### What was changed in `prepare_mission.py` (v6)

- **PASS 1.5 `refine_poses_mosaic()`** (plan item #10): every posed sample is
  re-registered against a coarse cruise-only mean mosaic (masked NCC over a
  local search window, damped, quality-gated), iterated; **convergence gate**
  keeps refined poses only when the final-pass median correction < 0.25 m
  (small: 0.19→0.09 m converges & helps; medium: 0.85→0.51 m diverges & is
  rolled back automatically). `SAR_REF_ITERS=0` disables for A/B.
- **`finalize_wall_map()` rebuilt** on the projection method: spread →
  axis-projection peaks → per-lane support runs (gap-bridged 1 m, min 1.5 m,
  density-gated) → robust strong-line bbox filter → exterior lines nudged
  inward 6 px (smear bias is outward) → ends snapped to perpendicular walls →
  4 px rasterization → 600×600 crop centred on the wall bbox.

### Scores (offline harness; baseline was map IoU 0.000 everywhere)

| world  | map IoU | map IoU @best-shift | victims (est/GT) | mean dist |
|--------|---------|---------------------|------------------|-----------|
| small  | 0.052   | 0.183 @ (+0.65,−1.80) m | 4/4          | 1.71 m    |
| medium | 0.027   | 0.072 @ (−0.40,−2.60) m | 4/4 (denim=0, broad fallback) | 12.8 m |
| large  | 0.011   | 0.076 @ (+1.80,+3.55) m | 4/5          | 4.6 m     |

### Next steps, in value order

1. **Victims (15%): fix absolute pose accuracy** — the 1 m scoring radius needs
   <1 m error; currently 1.5–2 m on small even where detection is perfect.
   Most promising: treat the mat homography as a *projective* anchor (not
   similarity-reduced), and/or bundle-adjust with the mat's checkerboard;
   consider raising anchor weight vs chain edges for samples near the mat and
   solving scale from lidar-free cues (known mat size at multiple altitudes).
2. **Map centring**: best-shift shows 0.07–0.18 IoU is available from alignment
   alone — bbox centring is off by 1–3 m because exterior wall extents are
   incomplete. Improving exterior-run completeness (lower run threshold for
   outermost peaks, or closing the outline) directly converts shift-IoU into
   raw IoU.
3. **Medium world victim detector**: denim detector finds 0 detections on
   medium (different victim clothing?) — broad fallback lands 12.8 m off.
   Inspect medium dive frames; extend the precise detector's appearance gates.
4. **Large world count**: 4/5 estimates — one victim never detected; check its
   dive coverage.