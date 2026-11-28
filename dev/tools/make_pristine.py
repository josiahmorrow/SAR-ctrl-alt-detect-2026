"""Snapshot the shipped preflight's outputs once per world, for hermetic runs.

A mission is not independent of the one before it: robot1 overwrites
sim_logs/map_estimate.png with the learned map at the end of the mission, and
the next preflight then sees "outputs up-to-date" and inherits it. run_batch.py
therefore restores a clean snapshot of sim_logs/ before every run.

The snapshot is produced by the controller's OWN preflight code: the source of
`_analyse_both` and `_promote_side_outputs` is lifted out of proposed_solution.py
with ast and executed against prepare_mission + flyover_extraction redirected to
a cache directory. So the cache cannot drift from what the controller would do
(an earlier cache held only our extraction, not the shipped hybrid; see
logged history in dev/docs).

    python dev/tools/make_pristine.py [small medium large] [--force]

Cache: <env>/_eval/pristine/<world>/   (override with SAR_PRISTINE_DIR)
"""
import ast
import hashlib
import json
import os
import shutil
import sys


def _find_env_root(start):
    p = os.path.dirname(os.path.abspath(start))
    while True:
        if (os.path.isdir(os.path.join(p, "worlds")) and os.path.isdir(
                os.path.join(p, "controllers", "sar_marking_supervisor"))):
            return p
        q = os.path.dirname(p)
        if q == p:
            raise RuntimeError("environment root (worlds/ + supervisor) not found")
        p = q


REPO = _find_env_root(__file__)
SOL = os.path.join(REPO, "controllers", "proposed_solution")
SIM_LOGS = os.path.join(SOL, "sim_logs")
CACHE = os.environ.get("SAR_PRISTINE_DIR") or os.path.join(REPO, "_eval", "pristine")
if not os.path.isabs(CACHE):
    CACHE = os.path.join(REPO, CACHE)

REQUIRED = ("victim_location_estimates.csv", "map_estimate.png",
            "nav_grid.npy", ".last_recording")
# Never restored: learned grids are per-mission state; debug/ is diagnostics.
TRANSIENT = ("learned_robot1.npz", "learned_robot2.npz", "debug")
META = "_pristine_meta.json"


def _source_hash():
    """Content hash of everything the analysis depends on."""
    h = hashlib.sha256()
    for f in ("prepare_mission.py", "flyover_extraction.py", "route_planning.py",
              "flyover_dr.py"):
        with open(os.path.join(SOL, f), "rb") as fh:
            h.update(fh.read().replace(b"\r\n", b"\n"))
    for f in ("_analyse_both", "_promote_side_outputs"):
        h.update(_function_source(f).encode())
    return h.hexdigest()[:16]


def _function_source(name):
    path = os.path.join(SOL, "proposed_solution.py")
    src = open(path, encoding="utf-8").read()
    for node in ast.parse(src).body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return ast.get_source_segment(src, node)
    raise RuntimeError(f"{name} not found in proposed_solution.py")


def _modules():
    if SOL not in sys.path:
        sys.path.insert(0, SOL)
    import prepare_mission as pm
    import flyover_extraction as fx            # noqa: F401  (imported by _analyse_both)
    return pm


def is_current(world):
    dest = os.path.join(CACHE, world)
    try:
        meta = json.load(open(os.path.join(dest, META)))
    except (OSError, ValueError):
        return False
    return (meta.get("source_hash") == _source_hash()
            and all(os.path.exists(os.path.join(dest, f)) for f in REQUIRED))


def build(world, force=False):
    dest = os.path.join(CACHE, world)
    if not force and is_current(world):
        return dest
    print(f"[pristine] analysing {world} with the shipped preflight...", flush=True)
    pm = _modules()
    tmp = dest + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    pm.SIM_LOGS = tmp
    pm.CSV_OUT = os.path.join(tmp, "victim_location_estimates.csv")
    pm.MAP_OUT = os.path.join(tmp, "map_estimate.png")
    pm._LAST_REC = os.path.join(tmp, ".last_recording")
    mp4, imu = pm.find_recording(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    if mp4 is None:
        raise RuntimeError(f"no recording for {world}")
    ns = {"os": os}
    for f in ("_promote_side_outputs", "_analyse_both"):
        exec(compile(_function_source(f), f"proposed_solution.py:{f}", "exec"), ns)
    ns["_analyse_both"](pm, mp4, imu)
    missing = [f for f in REQUIRED if not os.path.exists(os.path.join(tmp, f))]
    if missing:
        raise RuntimeError(f"preflight left no {missing} for {world}")
    for t in TRANSIENT:
        p = os.path.join(tmp, t)
        shutil.rmtree(p, ignore_errors=True) if os.path.isdir(p) else (
            os.remove(p) if os.path.exists(p) else None)
    json.dump({"source_hash": _source_hash(), "world": world,
               "recording": os.path.abspath(mp4)},
              open(os.path.join(tmp, META), "w"), indent=1)
    shutil.rmtree(dest, ignore_errors=True)
    os.replace(tmp, dest)
    return dest


def restore(world):
    """Make sim_logs/ exactly the pristine snapshot for `world`."""
    src = build(world)
    os.makedirs(SIM_LOGS, exist_ok=True)
    for name in os.listdir(SIM_LOGS):
        if name == ".gitkeep":
            continue
        p = os.path.join(SIM_LOGS, name)
        shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
    for name in os.listdir(src):
        if name == META:
            continue
        s, d = os.path.join(src, name), os.path.join(SIM_LOGS, name)
        shutil.copytree(s, d) if os.path.isdir(s) else shutil.copy2(s, d)
    # The cache key's code stamp is prepare_mission.py's mtime, which a git
    # checkout changes without changing the code. The snapshot's content hash
    # was just verified, so re-stamp it rather than let the preflight re-analyse.
    pm = _modules()
    rec = os.path.join(SIM_LOGS, ".last_recording")
    first = open(rec).read().splitlines()[0]
    with open(rec, "w") as f:
        f.write(f"{first}\n{pm._code_stamp()}\n")
    for name in ("victim_location_estimates.csv", "map_estimate.png"):
        os.utime(os.path.join(SIM_LOGS, name))   # newer than the recording


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for w in args or ["small", "medium", "large"]:
        print(build(w, force="--force" in sys.argv))
