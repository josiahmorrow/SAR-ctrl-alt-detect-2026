"""Planner ablation on the kept stack (exp24). usage: make_ablation.py
Writes ps_nav4d.py (log-odds map only; old soft planner + wall-follow) and
ps_nav4e.py (binary map; hard inflated planner + route re-check; unknown
cost 1.0; wall-follow on)."""
import subprocess, sys
D = r"C:\Users\agcoo\projects\sar\_scratch"
py = sys.executable
subprocess.run([py, rf"{D}\make_nav1.py", rf"{D}\ps_kept.py", rf"{D}\_abl_nav1.py"], check=True)
subprocess.run([py, rf"{D}\make_nav4.py", rf"{D}\_abl_nav1.py", rf"{D}\_abl_nav4.py"], check=True)


def edit(src, pairs):
    for old, new in pairs:
        assert src.count(old) == 1, old
        src = src.replace(old, new)
    return src


s = open(rf"{D}\_abl_nav1.py", encoding="utf-8").read()
s = edit(s, [("HARD_WALL_FIRST     = True ", "HARD_WALL_FIRST     = False")])
open(rf"{D}\ps_nav4d.py", "w", encoding="utf-8", newline="").write(s)

s = open(rf"{D}\_abl_nav4.py", encoding="utf-8").read()
s = edit(s, [("NAV_LOGODDS       = True", "NAV_LOGODDS       = False"),
             ("NAV_UNKNOWN_COST    = 1.5", "NAV_UNKNOWN_COST    = 1.0"),
             ("WALL_FOLLOW          = False", "WALL_FOLLOW          = True ")])
open(rf"{D}\ps_nav4e.py", "w", encoding="utf-8", newline="").write(s)
print("ok")
