"""Offline: every report, its verdict, true distance to the nearest victim and
whether that victim was already found. usage: falses.py <glob-of-run-dirs>"""
import glob, json, math, os, sys

for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        for sup in sorted(glob.glob(os.path.join(d, "*_supervisor.json"))):
            s = json.load(open(sup))
            vs = s["victims"]
            found_at = {k: v["found_time"] for k, v in vs.items()}
            lines = []
            for m in s["messages"]:
                px, py = m["position"][0], m["position"][1]
                t = m["sent_timestamp"]
                ds = sorted((math.hypot(px - v["position"][0], py - v["position"][1]), k)
                            for k, v in vs.items())
                dn, kn = ds[0]
                prior = found_at[kn] is not None and found_at[kn] < t - 0.01
                tag = "T" if m.get("victim_found_verdict") else "F"
                why = ""
                if tag == "F":
                    why = "dup" if (dn < 1.0 and prior) else ("far" if dn < 2.0 else "phantom")
                lines.append(f"{m['robot_id'][-1]}@{t:.0f}s {tag} {kn} {dn:.2f}m {why}")
            print(f"{os.path.basename(d)}/{os.path.basename(sup)[:-17]}: "
                  f"{s['victims_found']}/{s['total_victims']} | " + "; ".join(lines))
