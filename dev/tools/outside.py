"""Offline: time each robot spends in geofence-return mode (from the first
'Outside building envelope' print to the next telemetry line that is back
inside the return box), per run. usage: outside.py <run_dir>..."""
import glob, os, re, sys
TEL = re.compile(r"^\[(robot\d)\] t=\s*([\d.]+) \w+\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)")
OUT = re.compile(r"^\[(robot\d)\] Outside building envelope at \(([-\d.]+),([-\d.]+)\).*returning to \(([-\d.]+),([-\d.]+)\)")
for d in sys.argv[1:]:
    tot = {}
    for lf in glob.glob(os.path.join(d, "*_tee", "proposed_solution_*.log")):
        last_t = {}
        open_ep = {}
        for l in open(lf, encoding="utf-8", errors="replace"):
            m = TEL.search(l)
            if m:
                rid, t, x, y = m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4))
                last_t[rid] = t
                ep = open_ep.get(rid)
                if ep and abs(x - ep[1]) < 0.6 and abs(y - ep[2]) < 0.6:
                    tot[rid] = tot.get(rid, 0.0) + t - ep[0]
                    open_ep.pop(rid)
                continue
            m = OUT.search(l)
            if m:
                rid = m.group(1)
                if rid not in open_ep:
                    open_ep[rid] = (last_t.get(rid, 0.0), float(m.group(4)), float(m.group(5)))
        for rid, ep in open_ep.items():          # never got back
            tot[rid] = tot.get(rid, 0.0) + last_t.get(rid, ep[0]) - ep[0]
    print(os.path.basename(d.rstrip("\\/")), " ".join(f"{k}={v:.0f}s" for k, v in sorted(tot.items())) or "-")
