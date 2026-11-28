"""Offline: for each missed victim, each robot's closest odometry approach and
whether a vision candidate (any denim with depth) was logged within 3 m.
Robot frame = world - origin offset from the supervisor log.
usage: missed.py <glob-of-run-dirs>"""
import glob, json, math, os, re, sys

TEL = re.compile(r"\[(robot\d)\] t=\s*([\d.]+) (\w+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\).*?conf=([\d.]+) depth=\s*([\w.]+).*?why=(\S+)\s+block=(\S+)")
OFF = re.compile(r"Coordinate offset from OriginMarker: \[([-\d.e]+), ([-\d.e]+)")
for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        for sup in sorted(glob.glob(os.path.join(d, "*_supervisor.json"))):
            s = json.load(open(sup))
            tee = sup[:-len("_supervisor.json")] + "_tee"
            off = None
            for lf in glob.glob(os.path.join(tee, "sar_marking_supervisor*.log")):
                m = OFF.search(open(lf, encoding="utf-8", errors="replace").read())
                if m:
                    off = (float(m.group(1)), float(m.group(2)))
            if off is None:
                continue
            tel = {}
            for lf in glob.glob(os.path.join(tee, "proposed_solution_*.log")):
                for line in open(lf, encoding="utf-8", errors="replace"):
                    m = TEL.search(line)
                    if m:
                        tel.setdefault(m.group(1), []).append(
                            (float(m.group(2)), float(m.group(4)), float(m.group(5)),
                             float(m.group(6)), m.group(7), m.group(8), m.group(9)))
            for vid, v in sorted(s["victims"].items()):
                if v["found"]:
                    continue
                vx, vy = v["position"][0] - off[0], v["position"][1] - off[1]
                parts = []
                for rid, rows in sorted(tel.items()):
                    best = min(rows, key=lambda r: math.hypot(r[1] - vx, r[2] - vy))
                    dmin = math.hypot(best[1] - vx, best[2] - vy)
                    near = [r for r in rows if math.hypot(r[1] - vx, r[2] - vy) < 3.0]
                    cand = [r for r in near if r[3] > 0.0]
                    blocks = sorted({r[6] for r in cand})
                    parts.append(f"{rid} min {dmin:.1f}m@{best[0]:.0f}s near3m {len(near)*0.5:.0f}s cand {len(cand)} {','.join(blocks)}")
                print(f"{os.path.basename(d)} {vid} ({vx:.1f},{vy:.1f}): " + " | ".join(parts))
