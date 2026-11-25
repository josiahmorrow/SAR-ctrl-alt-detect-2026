#!/usr/bin/env python3
"""Offline sweep of a weak-segment-drop threshold for the wall map.

Loads cached `spread_*.npy` mosaics (dumped by prepare_mission with
SAR_DUMP_SPREAD=1), re-runs extract_wall_segments, drops segments whose
length x strength falls below `frac` of the strongest in their axis, rebuilds
the 600x600 map exactly as finalize_wall_map does, and scores it with
eval_harness. Pure map-metric tuning — no video reprocessing.
"""
import os, sys, subprocess, re, importlib.util
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
PM = os.path.join(REPO, "controllers/proposed_solution/prepare_mission.py")
spec = importlib.util.spec_from_file_location("pm", PM)
pm = importlib.util.module_from_spec(spec); spec.loader.exec_module(pm)
from PIL import Image

SIM = os.path.join(REPO, "controllers/proposed_solution/sim_logs")


def build_map(segs):
    G, MP, t = pm.GRID_PX, pm.MAP_PX, pm.WALL_THICK_PX // 2
    grid = np.zeros((G, G), np.uint8)
    for x, y0, y1, _ in segs[0]:
        grid[min(y0, y1):max(y0, y1) + 1, max(x - t, 0):x + t] = 255
    for y, x0, x1, _ in segs[1]:
        grid[max(y - t, 0):y + t, min(x0, x1):max(x0, x1) + 1] = 255
    if not grid.any():
        return np.full((MP, MP), 255, np.uint8)
    ys, xs = np.nonzero(grid)
    cx = int(round((xs.min() + xs.max()) / 2.0))
    cy = int(round((ys.min() + ys.max()) / 2.0))
    x0 = min(max(cx - MP // 2, 0), G - MP)
    y0 = min(max(cy - MP // 2, 0), G - MP)
    crop = grid[y0:y0 + MP, x0:x0 + MP]
    wall = np.full((MP, MP), 255, np.uint8)
    wall[crop > 0] = 0
    return wall


def prune(segs, frac):
    if frac <= 0:
        return segs
    out = {0: [], 1: []}
    for axis in (0, 1):
        if not segs[axis]:
            continue
        wv = [(s[2] - s[1]) * s[3] for s in segs[axis]]
        mx = max(wv)
        out[axis] = [s for s, w in zip(segs[axis], wv) if w >= frac * mx]
        if not out[axis]:                       # never empty a whole axis
            out[axis] = [segs[axis][int(np.argmax(wv))]]
    return out


def score(world, wall):
    tmp = os.path.join(SIM, "_tune_map.png")
    Image.fromarray(np.stack([wall] * 3, -1)).save(tmp)
    wbt = os.path.join(REPO, f"worlds/{world}_world.wbt")
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools/eval_harness.py"),
                        "score", wbt, "--map", tmp,
                        "--csv", os.path.join(SIM, "victim_location_estimates.csv")],
                       capture_output=True, text=True)
    out = r.stdout
    iou = re.search(r"MAP  IoU\s+([\d.]+)\s+F1\s+([\d.]+)", out)
    wpx = re.search(r"wall px: est\s+(\d+)\s+gt\s+(\d+)", out)
    return (float(iou.group(1)), float(iou.group(2)),
            int(wpx.group(1)), int(wpx.group(2))) if iou and wpx else (0, 0, 0, 0)


if __name__ == "__main__":
    fracs = [0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40]
    for world in ("small", "medium", "large"):
        sp = os.path.join(SIM, f"spread_{world}_world_flyover.npy")
        if not os.path.exists(sp):
            print(f"{world}: no spread cached"); continue
        spread = np.load(sp)
        segs0 = pm.extract_wall_segments(spread)
        print(f"\n=== {world}_world  (raw {len(segs0[0])}V+{len(segs0[1])}H) ===")
        print(f"{'frac':>5} {'IoU':>7} {'F1':>7} {'estpx':>7} {'gtpx':>7} {'V+H':>7}")
        for frac in fracs:
            segs = prune(segs0, frac)
            wall = build_map(segs)
            iou, f1, estpx, gtpx = score(world, wall)
            print(f"{frac:>5.2f} {iou:>7.4f} {f1:>7.4f} {estpx:>7} {gtpx:>7} "
                  f"{len(segs[0])}+{len(segs[1])}")
