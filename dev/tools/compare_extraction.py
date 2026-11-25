"""Score ANY branch's victim-estimate extraction on all three worlds, offline.

The only merge candidate between our branch and Zeke's is his victim estimation:
on small_world his estimates scored 0.836 at 0.37 m mean error against our 0.532
at 1.45 m. Before considering a merge, the question that decides it is whether
that accuracy holds on all three practice worlds or only on the one his pipeline
was calibrated hardest against — because an in-sample fit that collapses off its
calibration set is worth nothing on unseen judging worlds.

This runs a given branch's prepare_mission.analyse()/save() per world into a temp
directory and scores the resulting CSV with our eval harness. No Webots, no
mission — just the extraction half.

    .venv/bin/python tools/compare_extraction.py \
        --pm /tmp/zeke_test/controllers/proposed_solution/prepare_mission.py \
        --label zeke
"""
import argparse
import importlib.util
import os
import sys
import tempfile

import numpy as np


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
sys.path.insert(0, os.path.join(REPO, "tools"))
import eval_harness as eh  # noqa: E402

WORLDS = ("small", "medium", "large")


def load_pm(path, sim_logs):
    """Import a prepare_mission module and point its outputs at sim_logs."""
    pkg_dir = os.path.dirname(os.path.abspath(path))
    sys.path.insert(0, pkg_dir)                 # its siblings must be importable
    spec = importlib.util.spec_from_file_location("pm_under_test", path)
    pm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pm)
    pm.SIM_LOGS = sim_logs
    pm.CSV_OUT = os.path.join(sim_logs, "victim_location_estimates.csv")
    pm.MAP_OUT = os.path.join(sim_logs, "map_estimate.png")
    for attr in ("_LAST_REC", "LAST_REC"):
        if hasattr(pm, attr):
            setattr(pm, attr, os.path.join(sim_logs, ".last_recording"))
    return pm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pm", required=True, help="path to a prepare_mission.py")
    ap.add_argument("--label", default="branch")
    args = ap.parse_args()

    print(f"=== {args.label}: {args.pm} ===")
    for world in WORLDS:
        video = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
        imu = os.path.join(REPO, "recordings", f"{world}_world_flyover.csv")
        scene = eh.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
        with tempfile.TemporaryDirectory() as tmp:
            pm = load_pm(args.pm, tmp)
            try:
                try:
                    victims, wall = pm.analyse(video, imu)
                except TypeError:
                    victims, wall = pm.analyse(video)
                pm.save(victims, wall)
            except Exception as exc:                       # noqa: BLE001
                print(f"  {world:<7} analyse failed: {type(exc).__name__}: {exc}")
                continue
            csv_path = os.path.join(tmp, "victim_location_estimates.csv")
            rows = []
            if os.path.exists(csv_path):
                for line in open(csv_path):
                    line = line.strip()
                    if line:
                        p = line.split(",")
                        rows.append((float(p[0]), float(p[1])))
            est = np.array(rows) if rows else np.zeros((0, 2))
            rep, _proxy = eh.victim_report(est, scene["victims"], scene["origin"])
            n_true = len(scene["victims"])
            print(f"  {world:<7} {len(est)} estimates for {n_true} real victims")
            print("\n".join("      " + ln for ln in rep.splitlines()))
        # each world gets a fresh module: some pipelines cache per-recording state
        sys.modules.pop("pm_under_test", None)


if __name__ == "__main__":
    main()
