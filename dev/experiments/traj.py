"""Offline: condensed timeline for one robot. usage: traj.py <run_dir> <robotN> [every_s] [t0] [t1]"""
import glob, os, re, sys

d, rid = sys.argv[1], sys.argv[2]
every = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
t0 = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
t1 = float(sys.argv[5]) if len(sys.argv) > 5 else 1e9
TEL = re.compile(r"t=\s*([\d.]+) (\w+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)\s+wp(\d+)\s*=\(\s*([-\d.]+),\s*([-\d.]+)\).*?why=(\S+)\s+block=(\S+).*?ir=(\S+) tilt=(\d+)")
for lf in glob.glob(os.path.join(d, "*_tee", "proposed_solution_*.log")):
    lines = open(lf, encoding="utf-8", errors="replace").read().splitlines()
    if not any(l.startswith(f"[{rid}] Mission started") for l in lines):
        continue
    nxt = -1.0
    for l in lines:
        if not l.startswith(f"[{rid}]"):
            continue
        m = TEL.search(l)
        if m:
            t = float(m.group(1))
            if t < t0 or t > t1 or t < nxt:
                continue
            nxt = t + every
            print(f"{t:6.1f} {m.group(2):5s} ({float(m.group(3)):6.2f},{float(m.group(4)):6.2f}) "
                  f"wp{m.group(5)}=({m.group(6)},{m.group(7)}) {m.group(8):16s} {m.group(9):18s} ir={m.group(10)} tilt={m.group(11)}")
        elif " t=" not in l and "SLIP" not in l:
            tt = nxt - every
            if t0 <= tt <= t1:
                print("       " + l[len(rid) + 3:][:120])
