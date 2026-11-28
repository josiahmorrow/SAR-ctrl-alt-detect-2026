"""to1: the hard waypoint time cap scales with the distance to the waypoint.
usage: make_to1.py <in> <out>

v2b medium M1 (accurate flyover estimates): robot1's first waypoint is
23 m from the start. It drives straight at it at about 0.42 m/s, is 6 m
short at 41 s, and the flat WAYPOINT_TIMEOUT_S = 40 s cap abandons it
("timeout - requeued") while it is still closing. It then leaves for
another waypoint and comes back later. A flat cap is a stuck detector for
near targets and a guillotine for far ones; the progress-based stall test
(no 0.3 m gain in WAYPOINT_STALL_S) already detects being stuck.

to1: budget = max(WAYPOINT_TIMEOUT_S, 15 s + distance at assignment /
0.30 m/s). 0.30 m/s is a conservative fraction of the measured 0.42 m/s
straight-line progress; nothing is world-keyed.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("""                self._wp_start_time = now
                self._wp_best_dist  = dist
""", """                self._wp_start_time = now
                self._wp_best_dist  = dist
                self._wp_budget     = max(WAYPOINT_TIMEOUT_S,
                                          15.0 + dist / 0.30)   # to1
""")
rep("            if stalled or now - self._wp_start_time > WAYPOINT_TIMEOUT_S:",
    "            if stalled or now - self._wp_start_time > getattr(\n"
    "                    self, '_wp_budget', WAYPOINT_TIMEOUT_S):")
open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
