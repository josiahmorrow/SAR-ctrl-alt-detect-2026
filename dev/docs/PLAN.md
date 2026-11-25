# PLAN.md — Google-Maps rebuild (solo, branch `Google-Maps-inspired`, 28/07/2026)

> The work plan: what we do, in what order, and when we stop. Architecture, scoring and
> exact output contracts live in `docs/ARCHITECTURE.md`. Deadline: **Friday 31/07/2026**,
> hard stop Thursday night. This branch carries the full `joe-solo` history — the proven
> extraction pipeline, `tools/eval_harness.py`, and the controller fixes are already here.
> The rebuild target is ONE thing: the controller's navigation/itinerary/reporting chain.

---

## Part 1 — Distilled lessons (paid for in five live runs; not re-litigated)

**Facts that are dictated, not chosen:**
- Coordinate frame: origin = ArUco marker-0 centre; **+X → marker 2, +Y → marker 1, Z up**
  (Z-up ENU). The CSV, the map and the report messages live in this frame. No code ever
  redefines it, whatever internal left/right language we use.
- Scoring: a victim counts ONLY if the robot's **true position is <1.0 m from the
  VICTIM_MARKER** when a valid channel-43 JSON arrives, and the victim isn't already
  found. The message's own position field is logged, never scored. False reports poison
  the confidence sub-score. Over-reporting CSV estimates is penalised.
- **Marker-offset trap:** victim markers sit up to ±1.3 m from the visible body
  (per-model offsets in `Victim.proto`). Parking beside the denim can still be >1 m from
  the marker. The CSV estimates are better marker proxies (~1 m on small_world) than the
  body is.

**Facts we verified the hard way:**
- Compass heading = `atan2(cx, cy)` from `"imu compass"` — using cz rotated the whole
  navigation frame ~90°. WHEEL_RADIUS = **0.043** (0.05 gave 16% odometry error; with
  the fix, odometry is ~cm-accurate over a mission). Motors allow 26 rad/s; **14 rad/s
  cruise (~0.48 m/s)** is stable with speed-scaled steering + rotate-in-place >1.2 rad.
  The old 5 rad/s could not physically visit multiple victims in 180 s.
- **Every victim model wears blue denim** (HSV ≈ [95,35,25]–[135,255,255]); furniture
  doesn't. Denim presence + depth measured on the denim region is the only reliable
  camera gate. Colour-only detection reports lounges.
- Extraction: victim CSV is good on small (0.76/0.90/1.05/3.08 m), drifted on medium
  (~10 m); IMU supplies exactly ONE trustworthy correction (global radial scale, apply
  only when s>1). Deliverable formats verified compliant; map window is **bbox-centred**
  (supervisor convention — not "origin at 300,300").

**Approaches that failed (measured — do not repeat):**
- Pure reactive lidar navigation: every small_world victim is wall-blocked from the
  start (proven by A* on ground truth); line-of-sight driving just pins robots on walls.
- Eroding the planning grid to "reopen doorways": at 0.10 m erosion only ~2.1k of 360k
  cells stayed wall — routes degenerated to straight lines. Erosion deletes walls.
- Painting single ~30 cm patches on lidar bumps: far too little to reroute around a
  multi-metre wall — the planner re-issued the same route into the same wall.
- Per-frame IMU position fusion / full similarity fits: regress everywhere.
- Reporting on camera sight at range (3.5 m), colour-only gating, 8 s staggered starts.

---

## Part 2 — The attack plan

### 1. Map gate — verify before anything drives
Render the flyover wall map and prove it's real, not phantom walls:
- Dev worlds: score vs ground truth with `tools/eval_harness.py` (no regressions).
- All worlds (incl. judged/unseen): **A*-reachability self-check** — every victim
  estimate must be reachable from the robot start on the planning grid. With soft-cost
  walls (below) this check nearly cannot fail; a huge route cost still flags a suspect map.
- Preflight keys off `getWorldPath()` (never a cached recording from another world).

### 2. Victim export — in the dictated mat frame
CSV as required: headerless, numeric `x,y` per real victim, mat frame. Already produced
by the ported pipeline; the gate is simply "format + count verified" per world.

### 3. Route planning (the controller rebuild)
- **3.1 Balanced split.** Victims divided into two groups, counts differing ≤1, chosen
  to minimise total A* route length (brute force — trivial at ≤5 victims). No naive
  sign/left-right split: on small_world all four victims share the same X sign.
- **3.2 Furthest-first anchoring.** Each robot's route is anchored on its furthest
  victim (crow flies), committed from t=0 so the far victim isn't stranded at the
  3-minute mark. Intermediate victims are ordered by position along the actual A* path
  (with ≤3 per robot, brute-force the order by total A* length, anchor last — optimal).
- **3.3 Soft-cost A\* (planner core).** Wall cells cost high-but-finite to cross instead
  of being impassable. Real walls repel routes; a phantom wall sealing a doorway gets
  crossed at its thinnest point instead of making a victim "unreachable". Clearance by
  inflation only — **never erosion**.
- **3.4 Two-way map learning while driving.** Lidar endpoints paint walls INTO the grid;
  the free space each ray passes through erases phantom walls OUT of it. Re-plan on
  change. The map converges toward truth from both sides.
- **3.5 The last metre (where the marks are).** On arrival: 360° scan → lock on denim →
  confirm with finite denim-region depth → drive to <0.8 m → **reposition toward the CSV
  waypoint estimate if it's within ~1.5 m of the denim (marker-offset countermeasure)** →
  report ONCE at 0.9. No colour-only reports, no reports from range.
- **3.6 Assist mode (later add-on, Phase 2+).** A finished robot takes the partner's
  last victim, with a "claimed target" message on ch-73 so they never converge on the
  same victim. Skews the even split slightly; a found victim (40% pool) outweighs split
  evenness (15% pool) — deliberate trade.

### 4. Speed and time budget
Cruise 14 rad/s, steering scaled to speed, rotate-in-place for sharp turns. Budget
~35–45 s per victim including approach (4 victims fit in 180 s with margin). The enemy
is dithering, not top speed: scan ≤6 s, homing timeout ~12 s, failed-homing cooldown 8 s.
**Endgame rule:** from t≈170 s, a robot within ~2 m of an UNVISITED victim estimate
sprints to it and reports at closest approach — at most ONE speculative report per robot,
only in the final 10 s. Capped confidence cost, 40%-pool upside.

### 5. Degrade ladder (judged worlds have no ground truth — fail soft, never crash)
| Failure | Response |
|---|---|
| Recording missing/unreadable | Empty CSV+map skipped; robots run sweep-search mission |
| Map fails sanity (wall density absurd) | Ignore map; lidar-only driving, straight routes + learning |
| Victim estimates absent/absurd count | Sweep-search pattern; camera homing does the finding |
| A* route cost absurdly high | Treat as no-map case for that victim |
| Homing fails repeatedly at one target | Cooldown, resume route, revisit if time allows |

### 6. Test protocol and submission
- One change-theme per Webots run. Joe runs (Reload → Play), pastes console +
  `log/SupervisorLog_*.json`. Local commit after every edit; push on Joe's word.
- Extraction changes: `eval_harness score` on all three worlds first — no regressions.
- **Phase gates:** (1) one `VICTIM FOUND` on small_world — nothing else matters until
  this; (2) 3–4/4 twice back-to-back, even split, zero false reports; (3) medium + large
  run 180 s clean with ≥1 found each; (4) Thursday: fresh-clone rehearsal, formats
  re-verified, push, email `cgonzalezarango AT swin DOT edu DOT au`, subject EXACTLY
  `2026 IEEE SMCS SAR Competition Submission`. **Hard stop Thursday night.**
- Contingency (decide Wednesday night, don't drift into it): if gate 1 won't close,
  strip to minimum — 4 waypoints, soft-cost A* routes, scan, denim homing, report.
  A tested unspectacular submission beats an ambitious broken one.
