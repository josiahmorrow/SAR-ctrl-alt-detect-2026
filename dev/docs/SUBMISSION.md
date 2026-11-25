# Submission procedure & rehearsal record

**Team:** UniSC · **Competition:** 2026 IEEE SMCS SAR, Phase 1
**Deadline:** Friday 31/07/2026 · **Team review meeting:** Thursday 30/07, 8 pm

---

## 1. Why the submission repo is NOT this repo

The competition README states the submission repository's *contents* are cloned
into `controllers/proposed_solution/`. Submitting this development repo would
therefore nest as `controllers/proposed_solution/controllers/proposed_solution/…`
and Webots would find **no controller at all** — an automatic zero regardless of
solution quality.

`tools/build_submission.sh` stages the correct shape: `proposed_solution.py`,
`prepare_mission.py`, `requirements.txt`, `readme.md` and an empty `sim_logs/`
at the repository ROOT.

```bash
bash tools/build_submission.sh          # refuses to build if DEBUG_VISION is True
cd submission && git init && git add -A && git commit -m "UniSC SAR Phase 1 submission"
# push to the submission repo, then email the link
```

## 1a. The rehearsal that saved the submission (30/07)

A fresh clone placed in the judges' layout **crashed both controllers**:

```
ModuleNotFoundError: No module named 'cv2'      # x2, then a blind mission
Victims Found: 0/4        OVERALL SCORE: 0.074/1.000
```

Cause: our venv shim looked only at `../../.venv`, the path in *our* development
tree. The competition README says the judges install the environment where our
repository's contents are cloned — i.e. **beside** `proposed_solution.py`. The
controller now searches every plausible location and does nothing when the
imports already work. Re-rehearsed from a clean clone with the venv beside the
controller: both controllers start, a victim is found, 0.415.

Nothing in local testing would ever have revealed this: in our tree the shim
works. Only a fresh clone in the judges' layout exposes it.

## 1b. Third rehearsal, 30/07 late — final code, fresh clone, judge layout

The shipped build was re-rehearsed after tonight's controller changes. A fresh
`git clone` of `submission/` placed at `controllers/proposed_solution/`, with its
own venv installed **beside** `proposed_solution.py` from `requirements.txt` only:

```
[preflight] World 'small_world' -> small_world_flyover.mp4
[robot1] Nav map: 600x600 @ 0.1 m/px, lidar-first (empty prior; flyover bbox set)
[robot1] Ready  –  2 video waypoints + 20 sweep waypoints
victims 1/4   OVERALL 0.549   (188 s wall)
```

Identical to the development tree's score, which is the point of the exercise:
nothing in our repo is load-bearing for our own solution.

One finding worth recording for anyone repeating this: our local copy of
`controllers/sar_marking_supervisor/sar_marking_supervisor.py` carries a
`../../.venv` import shim (added earlier so the supervisor could import numpy —
the framework Python has only `certifi`). A rehearsal tree therefore needs a
`.venv` at its root for the SUPERVISOR, quite separately from the venv the
controller needs beside itself. This is a harness detail only: the judges run
their own supervisor in their own environment, and nothing about it ships.

## 2. Rehearsal performed 29/07/2026

| Check | Result |
| --- | --- |
| Build sanity gates (syntax, no absolute paths, `DEBUG_VISION=False`) | pass |
| Staged file list — root-level controller, no `__pycache__`, no dev tooling | pass |
| Fresh `git clone` of the submission, placed in a judge-shaped tree | pass |
| `find_recording()` resolves the right flyover from that layout | pass |
| Controller module imports; outputs target `sim_logs/` | pass |
| Third-party imports all covered by `requirements.txt` (numpy, cv2, PIL) | pass |
| Vision-gate harness (11 checks) | 11/11 |

**Defect found and fixed by this rehearsal.** On a world with no matching
recording, the old fallback analysed *the newest available* flyover — i.e. a
different building — and submitted those victim estimates and wall map as our
graded answer, while sending robots to phantom coordinates. Recording selection
is now world-specific (exact → loose → sole-recording), and refuses to analyse
a foreign world: the mission then runs on lidar + camera alone and still
produces a map deliverable. This only ever manifests on unseen judging worlds,
which is exactly what we are marked on.

## 2b. THE SUBMISSION IS NOW THE HYBRID (branch `zeke-extraction-hybrid`)

Zeke's flyover extraction writes the two graded deliverables; our controller does
everything else. Rationale and full comparison in `docs/BRANCH_COMPARISON.md`.

Official marking server, hermetic runs, 31/07 early hours:

| World | Runs | Hybrid | Ours alone | Δ |
| --- | --- | --- | --- | --- |
| small | 5 | 0.483 / 0.505 / 0.505 / 0.483 / 0.210 | 0.549 / 0.555 | −0.06 (noisy both ways) |
| medium | 3 | **0.580** ×3 | 0.473 ×2 | **+0.107** |
| large | 4 | **0.635 / 0.646 / 0.646 / 0.635** | 0.392 / 0.392 / 0.394 | **+0.249** |

Mean of world means **≈0.55** against 0.473. Medium reached **Coordination 1.00** —
both robots scoring, for the first time in the project — and large's map
deliverable went 0.112 → 0.799.

Fresh-clone rehearsal in the judges' layout, no cached analysis, venv installed
from `requirements.txt` beside the controller:

```
[preflight] Analysing small_world_flyover.mp4…
[preflight] our extraction kept for nav_grid.npy only (CSV/map in sim_logs/uni_extract/)
[preflight] graded deliverables written by flyover_extraction (Zeke's pipeline)
[robot1] Video waypoints assigned: (2.8,6.0), (8.5,6.5)
[preflight] robot2: outputs ready — proceeding
victims 1/4   OVERALL 0.483   (300 s wall)
```

Both pipelines ran in their intended roles, the robot1-analyses/robot2-waits
protocol still holds with two pipelines, and the score matches the development
tree. Startup costs ~5 minutes of WALL clock for the two analyses; it costs no
simulation time, because Webots does not advance until the controller's first
`step()`.

**Fallback:** branch `Google-Maps-inspired`, verified mean 0.473, is what to submit
if anything about the hybrid looks wrong at the last minute.

## 3. Measured performance of `Google-Maps-inspired` alone (verified 30/07)

Merged branch `Google-Maps-inspired`, hermetic runs, large replicated (it is the
only non-deterministic world):

| World | Victims | Overall | Finding | Efficiency | Coordination | Extraction |
| --- | --- | --- | --- | --- | --- | --- |
| small | 1/4 | 0.549 | 0.607 | 0.20 | 0.50 | 0.670 |
| medium | 1/4 | 0.473 | 0.690 | 0.26 | 0.50 | 0.306 |
| large | 1/5 | 0.392 / 0.392 / 0.394 (n=3) | 0.54-0.70 | 0.26-0.37 | 0.50 | 0.208 |

Mean **0.472**. Confidence accuracy is 0.99 whenever we avoid a false report, which
is the most dependable part of the solution.

Note on large_world, updated 30/07 late: three consecutive runs of this build gave
0.392 / 0.392 / 0.394, but a fourth run of an almost identical build returned
**0.066** (0/5, seven tilt events, 11 % coverage). The tail is worse than the
0.388–0.471 spread recorded earlier, so treat any single large_world figure —
ours or a competitor's — as nearly uninformative.

Method notes that matter for anyone repeating this:
* **small/medium are deterministic** — one run per config is conclusive.
  **large is not**: three runs of one build gave 0.394/0.456/0.471.
* **Runs must be hermetic.** The mission overwrites `sim_logs/map_estimate.png`
  with the learned map, so the next run inherits it as its flyover prior unless
  a pristine analysis is restored first (`tools/run_batch.py` does this).
* The marking server needs SSL root certificates on macOS framework Python or it
  fails silently; `tools/mark_log.py` re-scores any saved log with both map
  images attached (the README's JSON-only example scores the map 0.000).

## 4. Submission email

Send to `cgonzalezarango@swin.edu.au`, subject **exactly**:

```
2026 IEEE SMCS SAR Competition Submission
```

Suggested body:

> Dear Dr González Arango,
>
> Please find below our team's submission for Phase 1 of the 2026 IEEE SMCS
> Search and Rescue Competition.
>
> Team: UniSC (University of the Sunshine Coast)
> Repository: <LINK>
>
> The repository contains `proposed_solution.py` (ground-robot controller for
> both robots), `prepare_mission.py` (flyover video analysis, triggered
> automatically before the control loop starts, so no separate preprocessing
> command is required), a pinned `requirements.txt` (Python 3.10+), and a
> readme describing the solution and environment replication.
>
> Kind regards,
> Josiah Morrow
> University of the Sunshine Coast

If the repository is private, invite the judges rather than relying on the link.

## 5. Pre-send checklist

- [ ] `DEBUG_VISION = False` (build script enforces)
- [ ] `bash tools/build_submission.sh` runs clean
- [ ] Fresh clone → judge layout → Webots run completes on at least one world
- [ ] Repository accessible to the judges (public, or invitation sent)
- [ ] Subject line character-exact
