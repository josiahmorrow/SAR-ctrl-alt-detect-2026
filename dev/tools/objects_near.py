"""List world objects (top-level .wbt nodes) within R metres of points in the
mat frame. Offline ground-truth inspection only.

    python dev/tools/objects_near.py large 3.0 "x,y" ["x,y" ...]
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_pristine import REPO          # noqa: E402
import eval_harness as EH               # noqa: E402

world, R = sys.argv[1], float(sys.argv[2])
pts = [tuple(map(float, p.split(","))) for p in sys.argv[3:]]
text = open(os.path.join(REPO, "worlds", f"{world}_world.wbt"), encoding="utf-8").read()
nodes = EH.Parser(EH._strip_comments(text)).parse_world()
W = EH.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
ox, oy = W["origin"][0], W["origin"][1]
for (px, py) in pts:
    print(f"--- within {R} m of mat ({px}, {py})")
    rows = []
    for n in nodes:
        t = n.get("translation")
        if not t or len(t) < 2:
            continue
        x, y = t[0] - ox, t[1] - oy
        d = math.hypot(x - px, y - py)
        if d <= R:
            size = n.get("size")
            rows.append((d, n.type, n.get("name"), x, y, t[2] if len(t) > 2 else 0, size))
    for d, typ, name, x, y, z, size in sorted(rows):
        print(f"   {d:4.1f} m  {typ:24} {str(name):28} at ({x:6.2f},{y:6.2f}) z={z:.2f} size={size}")

