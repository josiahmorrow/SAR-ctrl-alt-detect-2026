"""List every homing (final-approach) episode in DEBUG_VISION runs: when it
started, from what depth, how long it lasted, how it ended, and the robot's
true distance to the nearest victim marker at the end.

    python dev/tools/homing_episodes.py <tag> [<tag> ...]
"""
import glob
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_pristine import REPO, SOL          # noqa: E402
import eval_harness as EH                     # noqa: E402

LINE = re.compile(r"^\[(robot\d)\] t=\s*([\d.]+) (\w+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\).*?"
                  r"depth=\s*([\w.]+)")
REPORT = re.compile(r"^\[(robot\d)\] Victim reported")


def victims(world):
    W = EH.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    o = W["origin"]
    return [(v["marker_world"][0] - o[0], v["marker_world"][1] - o[1])
            for v in W["victims"]]


def main():
    for tag in sys.argv[1:]:
        rd = os.path.join(SOL, "dev", "eval", "runs", tag)
        for log in sorted(glob.glob(os.path.join(rd, "*_*.log"))):
            m = re.match(r"(\w+?)_(\d+)\.log$", os.path.basename(log))
            if not m:
                continue
            world = m.group(1)
            V = victims(world)
            ep = {}
            print(f"== {tag} {os.path.basename(log)}")
            for ln in open(log, encoding="utf-8", errors="replace"):
                ln = ln.strip()
                r = REPORT.match(ln)
                if r and r.group(1) in ep:
                    ep[r.group(1)]["reported"] = True
                    continue
                mm = LINE.match(ln)
                if not mm:
                    continue
                rid, t, st = mm.group(1), float(mm.group(2)), mm.group(3)
                x, y = float(mm.group(4)), float(mm.group(5))
                d = mm.group(6)
                if st == "HOME" and rid not in ep:
                    ep[rid] = dict(t0=t, d0=d, reported=False)
                elif st != "HOME" and rid in ep:
                    e = ep.pop(rid)
                    dv = min(math.hypot(x - a, y - b) for a, b in V)
                    print(f"   {rid} t={e['t0']:6.1f}-{t:6.1f} ({t - e['t0']:5.1f} s) "
                          f"from depth {e['d0']:>5}  -> "
                          f"{'REPORTED' if e['reported'] else 'aborted '}  "
                          f"end {dv:4.2f} m from nearest marker")


if __name__ == "__main__":
    main()
