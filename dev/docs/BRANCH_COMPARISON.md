# Which branch do we submit? — measured comparison, Thu 30/07 evening

Agreed in the 09:59 meeting (Josiah + Zeke; Bertus and Alexandra absent): each of
us runs the other's branch and we submit whichever is better. This is that test,
run on the **official marking server**, not by impression.

**Bottom line: submit ours.** Zeke's branch as pushed scores below ours on the one
world where it completed, and the machinery behind its practice-world results is
keyed to the practice worlds by *file name*, so it cannot contribute on unseen
judging worlds. Details below, including the parts of his work that ARE better
than ours and why they still shouldn't ship.

---

## 1. How this was measured

Zeke's branch was checked out into its own git worktree, untouched, with its own
`prepare_mission.py` preflight running exactly as he wrote it (`--no-pristine`,
so our flyover pipeline was not imposed on his). His `sar_marking_supervisor.py`
is byte-identical to ours, so both branches were graded by the same code.

Two configurations, because his branch modifies the simulator:

| Configuration | Worlds/protos | Represents |
| --- | --- | --- |
| ZEKE_ASIS | **his** modified worlds + protos | his own development conditions |
| ZEKE_PRISTINE | official worlds + protos restored from `main` | what the judges run |

## 2. Results

| Build | World | Victims | Overall | Finding | Effic. | Coord. | Extraction | Wall clock |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **ours** (HEAD) | small | 1/4 | **0.549** | 0.607 | 0.202 | 0.50 | 0.670 | 182 s |
| **ours** | medium | 1/4 | **0.473** | 0.690 | 0.26 | 0.50 | 0.306 | 163 s |
| **ours** | large | 1/5 | **0.392 / 0.392 / 0.394** | 0.54–0.70 | 0.26–0.37 | 0.50 | 0.208 | 208–431 s |
| Zeke ASIS | small | 1/4 | **0.510** | 0.438 | 0.338 | 0.50 | 0.698 | 468 s |
| Zeke ASIS | medium | 1/4 at 60 s | **no report** | — | — | — | — | 603 s timeout |
| Zeke ASIS | large | — | **no output at all** | — | — | — | — | 604 s timeout |
| Zeke PRISTINE | small | 0/4 at 120 s | **no report** | — | — | — | — | 605 s timeout |

Two of his three worlds produced no score inside a 10-minute wall-clock budget
(ours finish a 3-minute mission in ~3 minutes). His medium run had found 1/4 by
60 s of sim time and was still only at 120 s of sim time after ten minutes of
real time; his large run wrote **zero lines of output**.

**The simulator edits are load-bearing.** On his own worlds his small run found a
victim by ~60 s and finished with 0.510. With the official worlds and protos
restored — everything else identical — the same run had found **0/4 by the point it
reached 120 s of sim time** and produced no report at all inside the 10-minute
budget. Whatever his contact-joint changes do for stability, his solution's
behaviour depends on them, and they are not part of a submission.

The claim discussed in the meeting was 7/13 victims — "every victim on the small
map, half on medium". From the pushed branch I measured **1/4 on small**. That
result may live in newer local work: the branch's last commit is **25 June**, five
weeks old.

## 3. What his branch does that ours doesn't — and why it can't ship

### 3a. The practice-world profiles are keyed to the world's FILE NAME

`controllers/proposed_solution/practice_profiles.py` (439 lines) defines
`SMALL_WORLD_PROFILE`, `MEDIUM_WORLD_PROFILE`, `LARGE_WORLD_PROFILE`, selected at
runtime by:

```python
def profile_for_world(world_path):
    return PROFILES_BY_WORLD.get(os.path.basename(world_path))
```

Each profile carries hardcoded coordinates and timings — e.g. small_world's
`post_arrival_report_xy=(8.454108, 6.489287)` (victim4 sits at (8.47, 6.54) in
mat coordinates), `adjusted_target=(1.32, -2.45)`, arrival gates such as
`x ∈ [1.00, 1.25], y ∈ [5.35, 5.45]` with `earliest_time_s=168.0`, and medium's
`adjusted_target=(10.147987, -25.860052)`.

To be fair to Zeke, none of this is concealed — the module docstring says
"Named practice-world calibration profiles", and the methods say "For small_world
only" and "reproduce the physical corridor shape from the validated lower-pair
run". It is honest, documented practice-world calibration.

But the judging worlds will not be named `small_world.wbt`. `profile_for_world()`
returns `None`, every profile-driven behaviour switches off, and whatever fraction
of his practice results came from these corridors, adjusted targets and timed
report gates is simply absent. That is the decisive fact for a submission graded
on unseen scenarios.

### 3b. His branch modifies the simulator, and the simulator isn't ours to send

`worlds/*.wbt` gain a `contactProperties` block (`maxContactJoints` 512 → 8192
for victim/floor contacts) and both `protos/Rosbot.proto` and `protos/Victim.proto`
gain contact materials. The intent is clear from the branch name —
`sar-webots-stability` — and Webots really does spam contact-point warnings on
these worlds.

The submission, though, is only `proposed_solution.py` + `prepare_mission.py`
cloned into the judges' `controllers/proposed_solution/`. Their worlds and protos
are theirs. So any stability his edits buy is development-only, and if his control
loop's timing was tuned against the modified physics, the tuning is being applied
to a different simulator at judging.

### 3c. His victim ESTIMATE extraction is genuinely better than ours on small

This is the one place he is clearly ahead, and it deserves saying plainly:

| small_world | victim estimate score | mean location error | map score | extraction |
| --- | --- | --- | --- | --- |
| ours | 0.532 | 1.45 m | 0.808–0.892 | 0.670–0.712 |
| Zeke | **0.836** | **0.37 m** | 0.560 | 0.698 |

His victim positions are four times more accurate than ours. Our better wall map
cancels it out in the extraction total, but on victim estimates alone he wins
decisively — and accurate estimates also mean the robot is sent to a point it can
actually park within 1.0 m of, which is our binding constraint.

So why not take it? Because of how it is produced. His `prepare_mission.py`
projects victim positions from the flyover **IMU** track, with scale constants
selected by recording duration (`imu_projection_params(duration_s)`:
"long recordings in the practice set follow a deeper route") and calibrated
against these three recordings. We tested the same idea on our own pipeline and
rejected it by measurement: IMU altitude/scale is dead-reckoned, and fitting it
took our small-world error from 1.45 m to 5.51 m. An in-sample fit to three known
recordings is exactly the thing that looks best on the practice set and
transfers worst.

Our victim estimates come from ArUco-calibrated visual odometry with loop
closure — worse on the practice worlds, but not fitted to them.

## 3d. MEASURED: the hybrid, and it is a large win

Built on branch `zeke-extraction-hybrid` in the early hours of 31/07 and measured
on the official server. His `flyover_extraction.py` (+ `route_planning.py`) writes
both graded deliverables; our `prepare_mission.py` runs only for `nav_grid.npy`
(planning grid and flyover bbox), with its CSV/map diverted to
`sim_logs/uni_extract/`; our controller is otherwise unchanged.

| World | Baseline (ours) | Hybrid | Δ |
| --- | --- | --- | --- |
| small | 0.549 / 0.555 | 0.483 · 0.210 (see below) | worse, and noisy |
| medium | 0.473 ×2 | **0.580 ×3** | **+0.107** |
| large | 0.392 / 0.392 / 0.394 | **0.635 / 0.646 / 0.646 / 0.635** | **+0.249** |

Where the gains come from, medium and large respectively:

- **Victim estimates.** 0.866 and 0.746, against effectively 0 and 0.303 — mean
  error 0.30 m and 0.61 m against 5.06 m and 3.05 m.
- **Coordination on medium reached 1.00** — the first time all week that BOTH
  robots scored a victim. It had been pinned at 0.50 in every world. Accurate
  waypoints are what unlocked it: 2/4 victims found instead of 1/4.
- **large's map deliverable 0.112 → 0.799.** His flyover map is simply far better
  than ours there, and at 13-16 % coverage our learned map never publishes.
- **Efficiency on large 0.26-0.37 → 0.468**, because the robots stop spending the
  mission driving at phantom coordinates.

**small is the exception and stays honest.** Its single find is a coin flip in
BOTH builds — of five hybrid runs, four found one victim and one found none (that
run had a near-flip, tilt 25°, and every waypoint timed out). The map deliverable
was also a lottery while the learned-map override was enabled: 0.906 / 0.144 /
0.144 at 45-47 % coverage, same build. Disabling the override (see the constant's
comment in the controller) trades a 0.14/0.91 gamble for a stable ~0.56 and makes
small 0.483 whenever a victim is found.

Mean of world means: **≈0.55 hybrid vs 0.473 baseline.**

## 4. Recommendation

**Submit the hybrid** (`zeke-extraction-hybrid`): his extraction, our controller.
Mean ≈0.55 against 0.473 for ours alone, and the two big gains (medium +0.107,
large +0.249) replicated across three and four runs respectively.

An earlier version of this document recommended submitting our branch and taking
nothing from his. That was correct about his *controller* — two of three worlds
produced no score, the profiles switch off by world name, the simulator edits
don't ship — and wrong about his *extraction*, which is separable from both and
much better than ours. The hybrid is what the measurements support.

1. **Ship both halves credited.** `flyover_extraction.py` carries a provenance
   header naming him, the measurements, and the known risk.
2. **Do NOT ship `practice_profiles.py`** (hardcoded coordinates keyed to world
   file names) or his world/proto edits. The build script fails if either the
   profile module is ever imported or his two files go missing.
3. **Keep `Google-Maps-inspired` as the fallback** — verified 0.473 mean, and it
   is the branch to submit if the hybrid shows any problem in final rehearsal.
4. **Residual risk, stated plainly:** his positions come from an IMU projection
   with constants fitted to our three practice recordings. If the judging worlds
   break it, the hybrid could score *below* our own branch, because our
   extraction is what it displaces. The counter-evidence is that it holds at
   0.30-0.61 m across three quite different routes while ours ranges 1.45-5.06 m.
5. **Phase 2:** a generalising version of accurate victim localisation is the
   single highest-value piece of work available to us, and these numbers are the
   proof of what it is worth — Coordination went from 0.50 to 1.00 on medium on
   the strength of estimate accuracy alone.

## 5. Reproducing this

```bash
git worktree add /tmp/zeke_test origin/zeke/sar-webots-stability
cp tools/run_batch.py /tmp/zeke_test/tools/          # his branch has no runner
cp -R .venv /tmp/zeke_test/.venv
cd /tmp/zeke_test
.venv/bin/python tools/run_batch.py --world all --runs 1 --tag ZEKE_ASIS --no-pristine
git checkout main -- worlds protos                   # official simulator
.venv/bin/python tools/run_batch.py --world small --runs 1 --tag ZEKE_PRISTINE --no-pristine
```

`--no-pristine` is what keeps the test fair: it leaves `sim_logs/` alone so the
foreign branch's own preflight runs as its author intended.
