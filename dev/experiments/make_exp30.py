"""exp30: no back-to-back retry of a stalled flyover waypoint.
usage: make_exp30.py <in> <out>

b180 / exp29 large L0 robot1: its last flyover waypoint stalls (35 s), is
"requeued at the end of the video leg", which is the very next slot, and
stalls again (35 s): 70 of 180 s on one unreachable target. The requeue
exists so the map can learn more in between; when nothing lies in between,
the retry sees the same map. exp30 requeues only if at least one other
flyover waypoint will be attempted first; otherwise the waypoint is
abandoned (and broadcast, so the partner may adopt it from its own side).
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)
rep("""                if (self.target_idx < self._n_video_wps
                        and self._wp_retries.get(key, 0) < 1):""",
    """                if (self.target_idx < self._n_video_wps - 1   # exp30
                        and self._wp_retries.get(key, 0) < 1):""")
open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
