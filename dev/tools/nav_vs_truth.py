"""
Render what the navigation stack actually sees vs ground truth, in the
origin-anchored nav frame:

  black  = ground-truth walls (parsed from the .wbt, mat frame)
  red    = flyover-extracted walls (nav_grid.npy - what A* plans on)
  blue / dark-orange = robot1 / robot2 soft-cost A* routes
  green o = GT victim markers      magenta x = CSV victim estimates

    python3 tools/nav_vs_truth.py worlds/small_world.wbt
"""
import importlib.util
import math
import os
import sys

import numpy as np
import cv2  # noqa: F401


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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = _find_env_root(__file__)
sys.path.insert(0, HERE)                       # eval_harness
sys.path.insert(0, os.path.join(HERE, "vision_gate_test"))

import eval_harness as EH

spec = importlib.util.spec_from_file_location(
    "ps_nvt", os.path.join(REPO, "controllers", "proposed_solution",
                           "proposed_solution.py"))
ps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps)
from controller import Robot  # noqa: E402

SIM_LOGS = os.path.join(REPO, "controllers", "proposed_solution", "sim_logs")
SCALE = 0.05          # render resolution m/px
G     = 1200          # render grid (60 m)


def w2px(x, y):
    return int(round(G / 2 + x / SCALE)), int(round(G / 2 - y / SCALE))


def main():
    wbt = sys.argv[1] if len(sys.argv) > 1 else "worlds/small_world.wbt"
    world = EH.load_world(os.path.join(REPO, wbt))
    ox, oy = world["origin"][0], world["origin"][1]

    canvas = np.full((G, G, 3), 255, np.uint8)

    # Flyover walls (red) — the exact grid the controller plans on.
    nav = np.load(os.path.join(SIM_LOGS, "nav_grid.npy"))
    f = int(round(G / nav.shape[0]))
    red = np.kron((nav > 0), np.ones((f, f), bool)) if f > 1 else (nav > 0)
    canvas[red] = (0, 0, 255)

    # Ground-truth walls (black), mat frame.
    for wall in world["walls"]:
        if wall["pos"][2] > 1.0:
            continue
        pts = [w2px(cx - ox, cy - oy) for cx, cy in EH.wall_corners(wall)]
        cv2.fillPoly(canvas, [np.array(pts, np.int32)], (0, 0, 0))

    # Controller + routes on the CURRENT nav grid.
    bots = {}
    for rid in ("robot1", "robot2"):
        bot = ps.SARRobot(Robot(rid))
        grid = np.load(os.path.join(SIM_LOGS, "nav_grid.npy"))
        ff = max(1, int(round(ps.NAV_RES / 0.05)))
        n = (grid.shape[0] // ff)
        occ = grid[:n * ff, :n * ff].reshape(n, ff, n, ff).max(axis=(1, 3))
        bot._nav_wall = (occ > 0).astype(np.uint8)
        bot._nav_prior = bot._nav_wall.copy()
        bot._nav_seen = np.zeros_like(bot._nav_wall)
        bot._nav_N = n
        bot._nav_cost = None
        bot._cost_dirty = True
        bot._route_cost_cache = {}
        bots[rid] = bot

    with open(os.path.join(SIM_LOGS, "victim_location_estimates.csv")) as fh:
        vids = [tuple(map(float, ln.split(","))) for ln in fh if ln.strip()]

    assign = bots["robot1"]._balanced_split(vids)
    colours = {"robot1": (255, 0, 0), "robot2": (0, 140, 255)}
    for rid, group in assign.items():
        bot = bots[rid]
        bot.x, bot.y = ps.START_POS[rid]
        for (gx, gy) in group:
            cells, cost = bot._astar(bot._w2c(bot.x, bot.y), bot._w2c(gx, gy))
            print(f"{rid} ({bot.x:5.2f},{bot.y:5.2f})->({gx:5.2f},{gy:5.2f})"
                  f" cost={cost:8.1f} cells={len(cells) if cells else 0}")
            if cells:
                pts = [w2px(*bot._c2w(r, c)) for r, c in cells]
                cv2.polylines(canvas, [np.array(pts, np.int32)], False,
                              colours[rid], 3)
            bot.x, bot.y = gx, gy
        cv2.circle(canvas, w2px(*ps.START_POS[rid]), 8, colours[rid], -1)

    for v in world["victims"]:
        mx, my = v["marker_world"][0] - ox, v["marker_world"][1] - oy
        cv2.circle(canvas, w2px(mx, my), 10, (0, 160, 0), 3)
    for (x, y) in vids:
        px, py = w2px(x, y)
        cv2.drawMarker(canvas, (px, py), (255, 0, 255), cv2.MARKER_TILTED_CROSS,
                       18, 3)

    out = os.path.join(HERE, "nav_vs_truth.png")
    ys, xs = np.nonzero((canvas != 255).any(axis=2))
    pad = 40
    crop = canvas[max(ys.min() - pad, 0):ys.max() + pad,
                  max(xs.min() - pad, 0):xs.max() + pad]
    cv2.imwrite(out, crop)
    print("wrote", out)


if __name__ == "__main__":
    main()
