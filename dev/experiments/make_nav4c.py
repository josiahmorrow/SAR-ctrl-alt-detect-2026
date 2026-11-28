"""nav4c (on nav4): learn walls as fast as the old map did, keep log-odds
clearing. usage: make_nav4c.py <in ps_nav4.py> <out>

nav4's startup spin learned 69-86 wall cells where the old binary map
learned ~1000: a cell needed two hits (LO_HIT 0.9 vs LO_OCC 1.0) from only
90 sampled beams, so the first routes were planned on an almost empty map
and drove into walls the robot had already scanned (nav4 small s1 robot2:
straight south into the wall between it and v2, 30 s lost; robot1 left the
building). One hit now makes a wall (LO_HIT 1.2) and one clear pass-through
removes it again (1.2 - 0.4 < 1.0), and 180 beams are sampled per update.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("LO_HIT            = 0.9", "LO_HIT            = 1.2    # nav4c: one hit makes a wall")
rep("LIDAR_MAP_BEAMS   = 90 ", "LIDAR_MAP_BEAMS   = 180")
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
