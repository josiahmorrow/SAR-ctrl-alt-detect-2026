"""List world objects (all nodes, nested included) within R metres of points
in the mat frame. Offline ground-truth inspection only.

    python dev/tools/objects_near.py large 1.5 "x,y" ["x,y" ...]
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
path = os.path.join(REPO, "worlds", f"{world}_world.wbt")
nodes = EH.Parser(open(path, encoding="utf-8").read()).parse_world()
W = EH.load_world(path)
ox, oy = W["origin"][0], W["origin"][1]
SKIP = {"Shape", "Appearance", "PBRAppearance", "ImageTexture", "TextureTransform",
        "Material", "Box", "Cylinder", "Sphere", "Mesh", "IndexedFaceSet",
        "Coordinate", "TextureCoordinate", "Physics", "Group", "Transform", "Pose"}
for (px, py) in pts:
    print(f"--- within {R} m of mat ({px}, {py})")
    rows = []
    for n, wt, _ in EH.walk(nodes):
        if n.type in SKIP:
            continue
        x, y = wt[0] - ox, wt[1] - oy
        d = math.hypot(x - px, y - py)
        if d <= R:
            rows.append((d, n.type, n.get("name"), x, y, wt[2], n.get("size")))
    for d, typ, name, x, y, z, size in sorted(rows)[:25]:
        print(f"   {d:4.1f} m  {typ:24} {str(name):28} at ({x:6.2f},{y:6.2f}) z={z:.2f} size={size}")
