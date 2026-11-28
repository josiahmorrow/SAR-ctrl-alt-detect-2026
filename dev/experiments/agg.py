"""Aggregate per-run CSVs <exp>_<W><seed>.csv (parN) plus legacy multi-row
CSVs into a per-experiment, per-world table. usage: agg.py exp [exp ...]
Legacy: 'exp08' reads exp08.csv; '<exp>S'/'<exp>M' CSVs also read."""
import csv, glob, os, sys, statistics as st
RUNS = os.path.join(os.path.expanduser("~"), "projects", "sar", "sar-env",
                    "controllers", "proposed_solution", "dev", "eval", "runs")

def rows_for(exp):
    out = []
    for p in glob.glob(os.path.join(RUNS, f"{exp}_*.csv")) + \
             [os.path.join(RUNS, f"{exp}{s}.csv") for s in ("", "S", "M")]:
        if not os.path.exists(p):
            continue
        for r in csv.DictReader(open(p)):
            out.append(r)
    return out

for exp in sys.argv[1:]:
    rows = rows_for(exp)
    for world in ("small", "medium", "large"):
        rs = [r for r in rows if r["world"] == world and r["victims_found"] != ""]
        if not rs:
            continue
        rs.sort(key=lambda r: int(r["seed"] or 0))
        f = [int(r["victims_found"]) for r in rs]
        fa = [int(r["false_reports"] or 0) for r in rs]
        tot = int(rs[0]["victims_total"])
        seeds = " ".join(f"s{r['seed'] or 0}:{r['victims_found']}" for r in rs)
        print(f"{exp:8s} {world:6s} n={len(rs)} mean found {st.mean(f):.2f}/{tot} "
              f"({100*st.mean(f)/tot:.0f}%)  sd {st.pstdev(f):.2f}  "
              f"false {st.mean(fa):.2f}  [{seeds}]")
