"""exp12: no marker-offset reposition. usage: make_exp12.py <in> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
old = "REPOSITION_MAX_M    = 2.2   #"
assert src.count(old) == 1
src = src.replace(old, "REPOSITION_MAX_M    = 0.0   # exp12: reposition OFF (was 2.2).")
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
