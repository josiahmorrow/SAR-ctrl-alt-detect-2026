"""
Offline route replay: run the REAL controller's soft-cost A* on the real
sim_logs/nav_grid.npy, from each robot's start to its balanced-split victims,
and render the routes over the wall map.

    python3 tools/route_replay.py            # writes tools/route_replay.png

Validates PLAN gates offline (no Webots): every victim estimate reachable,
route costs sane, routes passing through doorways rather than walls.
"""
import math
import os
import sys

import numpy as np
import cv2  # noqa: F401  (pre-import so the controller's venv shim is ignored)


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
sys.path.insert(0, os.path.join(HERE, "vision_gate_test"))     # controller stubs

import importlib.util

spec = importlib.util.spec_from_file_location(
    "ps_replay", os.path.join(REPO, "controllers", "proposed_solution",
                              "proposed_solution.py"))
ps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps)

from controller import Robot  # noqa: E402  (stub)

SIM_LOGS = os.path.join(REPO, "controllers", "proposed_solution", "sim_logs")


def load_csv():
    path = os.path.join(SIM_LOGS, "victim_location_estimates.csv")
    pts = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                x, y = line.split(",")
                pts.append((float(x), float(y)))
    return pts


def main():
    grid = np.load(os.path.join(SIM_LOGS, "nav_grid.npy"))
    vids = load_csv()
    bots = {}
    for rid in ("robot1", "robot2"):
        bot = ps.SARRobot(Robot(rid))
        bot._nav_wall  = (grid > 0).astype(np.uint8)
        bot._nav_N     = grid.shape[0]
        bot._nav_cost  = None
        bot._cost_dirty = True
        bot._route_cost_cache = {}
        bots[rid] = bot

    assign = bots["robot1"]._balanced_split(vids)
    print("Balanced split:")
    for rid, group in assign.items():
        print(f"  {rid}: " + " -> ".join(f"({x:.2f},{y:.2f})" for x, y in group))

    canvas = np.full((grid.shape[0], grid.shape[1], 3), 255, np.uint8)
    canvas[grid > 0] = (0, 0, 0)
    colours = {"robot1": (0, 0, 255), "robot2": (255, 0, 0)}   # BGR

    ok = True
    for rid, group in assign.items():
        bot = bots[rid]
        bot.x, bot.y = ps.START_POS[rid]
        for (gx, gy) in group:
            cells, cost = bot._astar(bot._w2c(bot.x, bot.y), bot._w2c(gx, gy))
            metres = (len(cells) - 1) * ps.NAV_RES if cells else float("inf")
            flag = ""
            if not cells:
                ok, flag = False, "  << NO ROUTE"
            elif cost > 4.0 * metres / ps.NAV_RES:
                flag = "  << SUSPECT (heavy wall crossing)"
            print(f"  {rid} ({bot.x:5.2f},{bot.y:5.2f}) -> ({gx:5.2f},{gy:5.2f})"
                  f"  len={metres:6.2f} m  cost={cost:8.1f}{flag}")
            if cells:
                for (r, c) in cells:
                    canvas[r, c] = colours[rid]
            bot.x, bot.y = gx, gy
        sr, sc = bot._w2c(*ps.START_POS[rid])
        cv2.circle(canvas, (sc, sr), 4, colours[rid], -1)
    for (gx, gy) in vids:
        r, c = bots["robot1"]._w2c(gx, gy)
        cv2.circle(canvas, (c, r), 5, (0, 180, 0), 2)

    out = os.path.join(HERE, "route_replay.png")
    cv2.imwrite(out, canvas)
    print(("ALL ROUTES OK  " if ok else "ROUTE FAILURES  ") + "-> " + out)


if __name__ == "__main__":
    main()
