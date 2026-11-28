"""Offline: vision features in the ~2 s before each report, labelled with the
supervisor verdict. usage: repfeat.py <glob-of-run-dirs>"""
import glob, json, os, re, sys

TEL = re.compile(r"\[(robot\d)\] t=\s*([\d.]+) \w+\s+pos=.*?conf=([\d.]+) depth=\s*([\w.]+) "
                 r"blob=([\d.]+) ratio=([\d.]+) h_med=\s*([\w.-]+) h_low=\s*([\w.-]+) "
                 r"wid=\s*([\w.]+) why=(\S+).*?tex_v=(\S+) tex_s=(\S+) thick=(\S+)")
rows = []
for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        for sup in sorted(glob.glob(os.path.join(d, "*_supervisor.json"))):
            s = json.load(open(sup))
            tee = sup[:-len("_supervisor.json")] + "_tee"
            tel = {}
            for lf in glob.glob(os.path.join(tee, "proposed_solution_*.log")):
                for line in open(lf, encoding="utf-8", errors="replace"):
                    m = TEL.search(line)
                    if m and m.group(10) == "candidate":
                        tel.setdefault(m.group(1), []).append(
                            (float(m.group(2)),) + tuple(m.group(i) for i in range(3, 14)))
            for msg in s["messages"]:
                rid, t = msg["robot_id"], msg["sent_timestamp"]
                near = [r for r in tel.get(rid, []) if t - 2.5 <= r[0] <= t + 0.1]
                if not near:
                    continue
                r = near[-1]
                rows.append((("T" if msg["victim_found_verdict"] else "F"),
                             os.path.basename(d), rid, t, r))
for v, d, rid, t, r in sorted(rows, key=lambda x: x[0]):
    print(f"{v} {d:10s} {rid} t={t:6.1f} conf={r[1]} d={r[2]} blob={r[3]} ratio={r[4]} "
          f"hmed={r[5]} hlow={r[6]} wid={r[7]} texv={r[9]} texs={r[10]} thick={r[11]}")
