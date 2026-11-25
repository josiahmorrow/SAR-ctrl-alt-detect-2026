"""Offline per-run diagnosis: robot tracks vs ground truth.

Needs a run made with SAR_DEBUG_VISION=1 (run_batch.py --debug-vision), whose
controller telemetry prints each robot's odometry pose every 0.5 s.

    python dev/tools/trajectory_plot.py <tag> <world> <n>
      -> dev/eval/runs/<tag>/<world>_<n>_traj.png  and a per-victim table:
         closest approach of each robot, when, in what state, and whether found.

Ground truth (walls, victim markers) comes from the .wbt via eval_harness and
from the supervisor log; this is offline evaluation only, never read by the
controller.
"""
import json
import math
import os
import re
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eval_harness as EH                      # noqa: E402
from make_pristine import REPO, SOL           # noqa: E402

LINE = re.compile(r"^\[(robot\d)\] t=\s*([\d.]+) (\w+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)"
                  r" wp(\d+)\s*=\(\s*([-\d.na]+),\s*([-\d.na]+)\) conf=([\d.]+)"
                  r" depth=\s*([\w.]+).*?why=(\S+)\s+block=(\S*)")
SCALE, G = 0.05, 1200


def w2px(x, y):
    return int(round(G / 2 + x / SCALE)), int(round(G / 2 - y / SCALE))


def main():
    tag, world, n = sys.argv[1], sys.argv[2], sys.argv[3]
    rd = os.path.join(SOL, "dev", "eval", "runs", tag)
    log = open(os.path.join(rd, f"{world}_{n}.log"), encoding="utf-8",
               errors="replace").read()
    sup = json.load(open(os.path.join(rd, f"{world}_{n}_supervisor.json")))
    W = EH.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    ox, oy = W["origin"][0], W["origin"][1]

    tracks = {}
    for ln in log.splitlines():
        m = LINE.match(ln.strip())
        if m:
            rid = m.group(1)
            tracks.setdefault(rid, []).append(dict(
                t=float(m.group(2)), state=m.group(3),
                x=float(m.group(4)), y=float(m.group(5)),
                conf=float(m.group(9)), why=m.group(11), block=m.group(12)))
    if not tracks:
        sys.exit("no DEBUG_VISION telemetry in this run (use --debug-vision)")

    canvas = np.full((G, G, 3), 255, np.uint8)
    for wall in W["walls"]:
        if wall["pos"][2] > 1.0:
            continue
        pts = [w2px(cx - ox, cy - oy) for cx, cy in EH.wall_corners(wall)]
        cv2.fillPoly(canvas, [np.array(pts, np.int32)], (0, 0, 0))
    csv = os.path.join(REPO, "_eval", "pristine", world,
                       "victim_location_estimates.csv")
    for ln in open(csv):
        if ln.strip():
            x, y = map(float, ln.split(","))
            cv2.drawMarker(canvas, w2px(x, y), (255, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 18, 3)
    col = {"robot1": (255, 0, 0), "robot2": (0, 140, 255)}
    for rid, tr in tracks.items():
        pts = np.array([w2px(p["x"], p["y"]) for p in tr], np.int32)
        cv2.polylines(canvas, [pts], False, col[rid], 2)
        cv2.circle(canvas, tuple(pts[0]), 7, col[rid], -1)

    victims = {}
    for vid, v in sup["victims"].items():
        vx, vy = v["position"][0] - ox, v["position"][1] - oy
        victims[vid] = (vx, vy, v)
        c = (0, 160, 0) if v["found"] else (0, 0, 220)
        cv2.circle(canvas, w2px(vx, vy), int(1.0 / SCALE), c, 2)   # 1 m radius
        cv2.putText(canvas, vid[-1], w2px(vx + 0.3, vy + 0.3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2)
    for m in sup.get("messages", []):
        px, py = m["position"][0] - ox, m["position"][1] - oy
        c = (0, 160, 0) if m.get("victim_found_verdict") else (0, 0, 255)
        cv2.drawMarker(canvas, w2px(px, py), c, cv2.MARKER_STAR, 22, 2)

    # Crop to the action.
    allp = [w2px(p["x"], p["y"]) for tr in tracks.values() for p in tr]
    allp += [w2px(v[0], v[1]) for v in victims.values()]
    xs, ys = zip(*allp)
    pad = 60
    x0, x1 = max(0, min(xs) - pad), min(G, max(xs) + pad)
    y0, y1 = max(0, min(ys) - pad), min(G, max(ys) + pad)
    out = os.path.join(rd, f"{world}_{n}_traj.png")
    cv2.imwrite(out, canvas[y0:y1, x0:x1])

    print(f"{world} run {n}: {sum(v[2]['found'] for v in victims.values())}"
          f"/{len(victims)} found  ->  {out}")
    print(f"{'victim':8} {'found':>6} {'t':>6}  " + "  ".join(
        f"{r+' min d (t, state)':>30}" for r in sorted(tracks)))
    for vid, (vx, vy, v) in victims.items():
        cells = []
        for rid in sorted(tracks):
            best = min(tracks[rid], key=lambda p: math.hypot(p["x"] - vx, p["y"] - vy))
            d = math.hypot(best["x"] - vx, best["y"] - vy)
            cells.append(f"{d:5.2f} m (t={best['t']:5.1f}, {best['state']:5})")
        ft = f"{v['found_time']:.1f}" if v["found"] else "-"
        print(f"{vid:8} {str(v['found']):>6} {ft:>6}  " + "  ".join(
            f"{c:>30}" for c in cells))
    for rid, tr in sorted(tracks.items()):
        whys = {}
        for p in tr:
            whys[p["why"]] = whys.get(p["why"], 0) + 1
        states = {}
        for p in tr:
            states[p["state"]] = states.get(p["state"], 0) + 1
        print(f"{rid}: states {states}  top gate reasons "
              f"{sorted(whys.items(), key=lambda kv: -kv[1])[:5]}")


if __name__ == "__main__":
    main()
