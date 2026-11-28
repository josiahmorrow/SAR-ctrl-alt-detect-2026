"""Offline: per run, per robot, seconds spent tilted (>=11 deg) and the time
the robot became permanently beached (tilted to the end). usage: beach.py <glob-of-run-dirs>"""
import glob, json, os, re, sys

TEL = re.compile(r"\[(robot\d)\] t=\s*([\d.]+) (\w+) pos=\(\s*([-\d.]+),\s*([-\d.]+)\).*tilt=(\d+)")
for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        for tee in sorted(glob.glob(os.path.join(d, "*_tee"))):
            sup = tee[:-4] + "_supervisor.json"
            try:
                s = json.load(open(sup))
                found = s["victims_found"]; tot = s["total_victims"]
                fals = sum(1 for m in s["messages"] if not m.get("victim_found_verdict"))
            except Exception:
                found = tot = fals = "?"
            out = []
            for lf in sorted(glob.glob(os.path.join(tee, "proposed_solution_*.log"))):
                rows = []
                for line in open(lf, encoding="utf-8", errors="replace"):
                    m = TEL.search(line)
                    if m:
                        rows.append((m.group(1), float(m.group(2)), int(m.group(6)),
                                     float(m.group(4)), float(m.group(5))))
                if not rows:
                    continue
                rid = rows[0][0]
                tilted = sum(1 for r in rows if r[2] >= 11) * 0.5
                # permanent beach: last time tilt < 11
                last_ok = max((r[1] for r in rows if r[2] < 11), default=0.0)
                end = rows[-1][1]
                beach = f"beached@{last_ok:.0f}s" if end - last_ok > 30 else "free"
                # stationary tail
                xs = [(r[3], r[4]) for r in rows]
                stat_from = end
                for r in reversed(rows):
                    if abs(r[3] - xs[-1][0]) > 0.05 or abs(r[4] - xs[-1][1]) > 0.05:
                        break
                    stat_from = r[1]
                tail = sorted(r[2] for r in rows if r[1] > last_ok)
                med = tail[len(tail) // 2] if tail else 0
                out.append(f"{rid}: tilted {tilted:.0f}s {beach} med{med} static-from {stat_from:.0f}s")
            print(f"{os.path.basename(d)}/{os.path.basename(tee)[:-4]}: found {found}/{tot} F{fals} | " + " | ".join(out))
