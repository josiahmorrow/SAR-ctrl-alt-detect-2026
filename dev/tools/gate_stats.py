"""Distribution of the vision gate's measurements for candidates that were
real victims vs not. Offline evaluation with ground truth.

A telemetry line is labelled REAL when the robot is within 2.0 m of a true
victim marker and the measured denim depth is <= 1.5 m (so the blob is most
likely that victim), FAR when the robot is > 3.0 m from every marker
(the blob cannot be a victim). Lines in between are ignored.

    python dev/tools/gate_stats.py <tag> [<tag> ...]
"""
import glob
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_pristine import REPO, SOL          # noqa: E402
import eval_harness as EH                     # noqa: E402

LINE = re.compile(r"^\[(robot\d)\] t=\s*([\d.]+) (\w+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\).*?"
                  r"conf=([\d.]+) depth=\s*([\w.]+) blob=([\d.]+) ratio=([\d.]+) "
                  r"h_med=\s*([-\w.]+) h_low=\s*([-\w.]+) wid=\s*([-\w.]+) why=(\S+)")


def markers(world):
    W = EH.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    o = W["origin"]
    return [(v["marker_world"][0] - o[0], v["marker_world"][1] - o[1])
            for v in W["victims"]]


rows = {"REAL": [], "FAR": []}
for tag in sys.argv[1:]:
    for log in glob.glob(os.path.join(SOL, "dev", "eval", "runs", tag, "*_*.log")):
        m = re.match(r"(\w+?)_\d+\.log$", os.path.basename(log))
        if not m:
            continue
        M = markers(m.group(1))
        for ln in open(log, encoding="utf-8", errors="replace"):
            mm = LINE.match(ln.strip())
            if not mm or mm.group(13) != "candidate":
                continue
            x, y = float(mm.group(4)), float(mm.group(5))
            depth = float(mm.group(7))
            if not math.isfinite(depth):
                continue
            dmin = min(math.hypot(x - a, y - b) for a, b in M)
            lab = "REAL" if (dmin < 2.0 and depth <= 1.5) else ("FAR" if dmin > 3.0 else None)
            if lab:
                rows[lab].append((float(mm.group(10)), float(mm.group(11)),
                                  float(mm.group(12)), depth, m.group(1), tag,
                                  round(x, 1), round(y, 1)))

for lab, rs in rows.items():
    print(f"{lab}: {len(rs)} candidate lines")
    if not rs:
        continue
    a = np.array([r[:4] for r in rs], float)
    for i, name in enumerate(("h_med", "h_low", "width_m", "depth")):
        col = a[:, i][np.isfinite(a[:, i])]
        if col.size:
            print(f"   {name:8} min {col.min():5.2f}  p10 {np.percentile(col, 10):5.2f}  "
                  f"median {np.median(col):5.2f}  p90 {np.percentile(col, 90):5.2f}  "
                  f"max {col.max():5.2f}")
    if lab == "FAR":
        where = {}
        for r in rs:
            where.setdefault((r[4], r[6], r[7]), 0)
            where[(r[4], r[6], r[7])] += 1
        print("   FAR candidates by place:", sorted(where.items(), key=lambda kv: -kv[1])[:8])
