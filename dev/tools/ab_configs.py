"""
Flip named controller constants for A/B experiments, run a batch, restore.

Webots is DETERMINISTIC: the same code on the same world yields byte-identical
scores (verified 30/07 — two runs, same 0.543). So repeats are worthless and a
single run per (config, world) is conclusive. Variance seen across yesterday's
runs was caused entirely by code changes, which means causes can be isolated
exactly by toggling one constant at a time.

    .venv/bin/python tools/ab_configs.py --config frontier-off --world medium
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile


def _find_env_root(start):
    # dev/ lives inside controllers/proposed_solution of the organisers' repo;
    # the environment root is the first ancestor holding worlds/ and the
    # untouched marking supervisor.
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
CTRL = os.path.join(REPO, "controllers", "proposed_solution", "proposed_solution.py")

# name -> {constant: new_value}
CONFIGS = {
    "baseline":        {},
    "frontier-off":    {"EXPLORE_FRONTIERS": "False"},
    "frontier-on":     {"EXPLORE_FRONTIERS": "True"},
    "stall-off":       {"WAYPOINT_STALL_S": "1000000.0"},
    "frontier+stall-off": {"EXPLORE_FRONTIERS": "False",
                           "WAYPOINT_STALL_S": "1000000.0"},
    "streak4":         {"REPORT_STREAK_N": "4"},
    "assist-off":      {"ASSIST_ENABLED": "False"},
    "assist-keep-failed": {"ASSIST_SKIP_FAILED": "False"},
    "cover15":         {"LEARNED_MIN_COVER": "0.15"},
    "wall300":         {"NAV_WALL_COST": "300.0"},
    "debug":           {"DEBUG_VISION": "True"},
    "region-off":      {"REGION_BLACKLIST": "False"},
    "wf-off":          {"WALL_FOLLOW": "False"},
    "wf-short":        {"WALL_FOLLOW": "True", "WF_NO_PROGRESS_S": "20.0",
                        "WF_MAX_S": "6.0"},
    "partnerfail-off": {"ASSIST_SKIP_OWN_ONLY": "False"},
    "wall120":         {"NAV_WALL_COST": "120.0"},
    "wall300-t25":     {"NAV_WALL_COST": "300.0", "WAYPOINT_TIMEOUT_S": "25.0"},
    "cover10":         {"LEARNED_MIN_COVER": "0.10"},
    # Bug2 post-mortem: the geometric leave test and the waypoint deadline
    # credit shipped together and lost small_world 0.551 -> 0.208 (twice).
    # Separate them: the credit delays abandonment, which delays the sweep leg
    # where small's only find actually happens.
    "bug2-nocredit":   {"WF_TIME_CREDIT_S": "0.0", "WF_MAX_S": "12.0"},
    # The credit result implies its own converse. Extending the waypoint deadline
    # by up to 20 s cost 0.343 because it delayed the sweep leg, which is where
    # small_world's only find happens — so SHORTENING the deadline should help
    # for the same reason. Cheap, and derived from a measurement, not a hunch.
    "timeout25":       {"WAYPOINT_TIMEOUT_S": "25.0"},
    "timeout15":       {"WAYPOINT_TIMEOUT_S": "15.0"},
    # Off-map diagnosis 31/07: at 3.5 m the fence never fires (robot2 orbited
    # the building 3.19 m out, un-escorted, for a third of the mission). 2.0
    # still clears the extraction's ~1.4 m bbox error on small.
    "fence2":          {"GEOFENCE_SLACK_M": "2.0"},
    "best-cover15":    {"EXPLORE_FRONTIERS": "False", "WAYPOINT_STALL_S": "1000000.0", "LEARNED_MIN_COVER": "0.15"},
}


def set_consts(mapping):
    src = open(CTRL).read()
    for k, v in mapping.items():
        pat = re.compile(rf"^({re.escape(k)}\s*=\s*)([^\s#]+)", re.M)
        if not pat.search(src):
            print(f"!! constant {k} not found", file=sys.stderr)
            sys.exit(2)
        src = pat.sub(rf"\g<1>{v}", src, count=1)
    open(CTRL, "w").write(src)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, choices=list(CONFIGS))
    ap.add_argument("--world", default="medium")
    args = ap.parse_args()

    # Refuse to run with uncommitted controller changes: this script restores
    # its backup afterwards, which on 30/07 silently reverted hand edits made
    # while a batch was in flight (and the next commit captured the reverted
    # state). Everything must be committed before an A/B run.
    dirty = subprocess.run(["git", "status", "--porcelain", CTRL],
                           cwd=REPO, capture_output=True, text=True).stdout
    if dirty.strip():
        print("!! controller has uncommitted changes — commit first:\n" + dirty,
              file=sys.stderr)
        sys.exit(3)

    # NOTE: restore with git, never a file copy. A copied backup is taken at
    # START of the run and written back at the END, so any edit made meanwhile
    # is silently reverted — that happened three times on 30/07, once losing a
    # committed change and invalidating a measurement. The dirty-tree check
    # above guarantees the committed state IS the intended state.
    try:
        set_consts(CONFIGS[args.config])
        subprocess.run([os.path.join(REPO, ".venv", "bin", "python"),
                        os.path.join(REPO, "tools", "run_batch.py"),
                        "--world", args.world, "--runs", "1",
                        "--tag", args.config], cwd=REPO, check=False)
    finally:
        subprocess.run(["git", "checkout", "--", CTRL], cwd=REPO, check=False)
        print(f"[ab] restored {CTRL} from git")


if __name__ == "__main__":
    main()
