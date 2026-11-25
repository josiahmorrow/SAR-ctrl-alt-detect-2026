# dev/ - post-competition development tooling

Nothing in `dev/` is part of the controller. The six shipped files at the repo
root (`proposed_solution.py`, `prepare_mission.py`, `flyover_extraction.py`,
`route_planning.py`, `requirements.txt`, `readme.md`) are the solution;
`dev/` is how it is measured.

## Layout (the judges' layout)

```
sar-env/                         clone of IEEE-SMCS/2026-ieee-smcs-competition-phase-1
  worlds/, recordings/           untouched
  controllers/sar_marking_supervisor/   untouched (scoring source of truth)
  controllers/proposed_solution/ <- this repo (SAR-ctrl-alt-detect-2026)
  .venv/                         dev venv; Webots' Python command points here
  log/                           supervisor output (SupervisorLog_*.json, Marked_*.txt)
  _eval/pristine/<world>/        cached flyover preflight snapshots
```

The organisers' example `proposed_solution/` is moved aside; `git -C sar-env diff
HEAD -- controllers/sar_marking_supervisor worlds protos` must stay empty.

## Setup (Windows)

```powershell
cd sar-env
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r controllers\proposed_solution\requirements.txt
Copy-Item controllers\proposed_solution\dev\tools\telemetry\sitecustomize.py .venv\Lib\site-packages\
# Webots > Tools > Preferences > General > Python command = <sar-env>\.venv\Scripts\python.exe
```

## Tools

| Tool | Status | Purpose |
|---|---|---|
| `tools/run_batch.py` | verified (Windows) | N hermetic missions per world; CSV of official + ground-truth outcome |
| `tools/make_pristine.py` | new | Snapshots the shipped hybrid preflight per world, using the controller's own `_analyse_both` |
| `tools/telemetry/sitecustomize.py` | new | Tees controller/supervisor stdout to files when `SAR_TEE_DIR` is set (Webots on Windows drops `--stdout`) |
| `tools/vision_gate_test/` | verified | Offline camera-gate regression test (11 checks) |
| `tools/eval_harness.py`, `mark_log.py`, `nav_vs_truth.py`, `route_replay.py` | copied, not yet re-verified | Offline scoring and replay |
| others | copied for history | Phase 1 experiment scripts |

## Provenance

`tools/` and `docs/` were copied on 25/11/2026 from the team development repo
(`abertus269/UniSC-SAR-Competition`, `main` at `48ee484`, the tree the
submission was built from). Each tool's `REPO` line was rewritten to find this
layout's environment root; `run_batch.py` was ported to Windows. The team
repo's `build_submission.sh` and `make_hybrid_pristine.py` were dropped: the
first built this repo, the second depended on a `/tmp` cache that no longer
exists and is replaced by `make_pristine.py`.

Ground truth (`.wbt`, supervisor logs) is read only by these offline tools,
never by the controller.

Post-competition experiment log: `docs/post_comp_log.md`.
