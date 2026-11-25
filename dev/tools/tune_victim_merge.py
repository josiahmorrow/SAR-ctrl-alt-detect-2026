#!/usr/bin/env python3
"""Offline experiment: merge denim (precise) + broad (recall) victim clusters.

Loads cached raw detections (dets_denim_*.npy / dets_broad_*.npy from a
SAR_DUMP_DETS run), tries clustering/merge strategies, and scores each against
the .wbt ground truth (found@1m, found@2m, mean matched dist, #estimates).
Pure clustering tuning — no video reprocessing.
"""
import os, sys, importlib.util, math
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
SIM = os.path.join(REPO, "controllers/proposed_solution/sim_logs")
pm = importlib.util.module_from_spec(
    importlib.util.spec_from_file_location("pm",
        os.path.join(REPO, "controllers/proposed_solution/prepare_mission.py")))
importlib.util.spec_from_file_location  # noqa
spec = importlib.util.spec_from_file_location("pm",
        os.path.join(REPO, "controllers/proposed_solution/prepare_mission.py"))
pm = importlib.util.module_from_spec(spec); spec.loader.exec_module(pm)
eh = importlib.util.module_from_spec(
    importlib.util.spec_from_file_location("eh",
        os.path.join(REPO, "tools/eval_harness.py")))
spec2 = importlib.util.spec_from_file_location("eh", os.path.join(REPO, "tools/eval_harness.py"))
eh = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(eh)


def cluster_votes(pts, r):
    """Greedy clusters returning [(cx, cy, votes)] (mirrors cluster_detections)."""
    if len(pts) == 0:
        return []
    arr = np.asarray(pts, float)
    used = np.zeros(len(arr), bool)
    out = []
    for i in range(len(arr)):
        if used[i]:
            continue
        d = np.linalg.norm(arr - arr[i], axis=1)
        m = d < r
        c = arr[m].mean(0)
        out.append((c[0], c[1], int(m.sum())))
        used[m] = True
    out.sort(key=lambda t: -t[2])
    return out


def gt_xy(world):
    wd = eh.load_world(os.path.join(REPO, f"worlds/{world}_world.wbt"))
    o = wd["origin"]
    return np.array([[v["marker_world"][0] - o[0], v["marker_world"][1] - o[1]]
                     for v in wd["victims"]])


def score(est, gt):
    if len(est) == 0:
        return 0, 0, 9.99, 0
    est = np.asarray(est, float)
    # nearest-GT distance for each est; greedy match GT->nearest est
    found1 = found2 = 0
    dists = []
    for g in gt:
        d = np.linalg.norm(est - g, axis=1).min()
        dists.append(d)
        found1 += d < 1.0
        found2 += d < 2.0
    return found1, found2, float(np.mean(dists)), len(est)


def merge_strategy(denim, broad, sep, min_votes_frac):
    """denim clusters + broad clusters that are >sep from all denim clusters
    and carry >= min_votes_frac of the broad top-cluster votes."""
    dv = pm.cluster_detections(list(map(tuple, denim)),
                               r=pm.DENIM_CLUSTER_R, div_r=pm.DENIM_DIVERSITY_R) \
        if len(denim) >= 6 else []
    bc = cluster_votes(broad, pm.CLUSTER_R_M)
    top = bc[0][2] if bc else 0
    add = []
    picked = list(dv)
    for cx, cy, v in bc:
        if v < max(1, min_votes_frac * top):
            continue
        if all(math.hypot(cx - px, cy - py) > sep for px, py in picked):
            picked.append((cx, cy)); add.append((cx, cy))
        if len(picked) >= pm.MAX_VICTIM_ESTIMATES:
            break
    return dv, picked


if __name__ == "__main__":
    for W in ("small", "medium", "large"):
        dp = f"{SIM}/dets_denim_{W}_world_flyover.npy"
        bp = f"{SIM}/dets_broad_{W}_world_flyover.npy"
        if not (os.path.exists(dp) and os.path.exists(bp)):
            print(f"{W}: no dets cached"); continue
        denim = np.load(dp); broad = np.load(bp)
        g = gt_xy(W)
        # baseline = current pipeline behaviour
        if len(denim) >= 6:
            base = pm.cluster_detections(list(map(tuple, denim)),
                                         r=pm.DENIM_CLUSTER_R, div_r=pm.DENIM_DIVERSITY_R)
        else:
            base = pm.cluster_detections(list(map(tuple, broad)))
        print(f"\n==== {W}_world  (GT {len(g)} victims; denim {len(denim)} broad {len(broad)}) ====")
        f1, f2, md, n = score(base, g)
        print(f"  baseline           found@1m {f1}/{len(g)}  @2m {f2}  mean {md:.2f}  n_est {n}")
        for sep in (2.0, 2.5):
            for mvf in (0.25, 0.4, 0.55):
                _, merged = merge_strategy(denim, broad, sep, mvf)
                f1, f2, md, n = score(merged, g)
                print(f"  merge sep={sep} mvf={mvf:<4} found@1m {f1}/{len(g)}  @2m {f2}  "
                      f"mean {md:.2f}  n_est {n}")
