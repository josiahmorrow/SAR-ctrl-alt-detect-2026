"""nav4b: the planner may not route outside the building envelope.
usage: make_nav4b.py <in ps_nav4.py> <out>

nav4 made known walls hard and unseen space cheap (1.5). Outside the
building everything is unseen, so the cheapest route between two rooms
became the way round the OUTSIDE of the building (nav4 small s1 robot1:
assisted toward (4.3,-4.6) from (9.4,3.4), planned east, left the building
and sat wedged at (12.3,2.5) for 450 s; no small run found victim4).
Cells outside the flyover envelope (bbox + NAV_ENV_SLACK_M, only when the
bbox passes the existing reliability gate) are hard-blocked for the hard
search and cost NAV_WALL_COST for the soft fallback.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("NAV_UNKNOWN_COST    = 1.5",
    "NAV_ENV_SLACK_M     = 1.5    # nav4b: planning envelope = bbox + this\n"
    "NAV_UNKNOWN_COST    = 1.5")

rep("""            self._nav_block = cv2.dilate(wall, kb) > 0""",
    """            self._nav_block = cv2.dilate(wall, kb) > 0
            out = self._nav_outside_mask() if NAV4 else None
            if out is not None:
                cost[out] = NAV_WALL_COST
                self._nav_block |= out""")

rep("""    def _path_blocked(self):""",
    '''    def _nav_outside_mask(self):
        """nav4b: cells beyond the building envelope (cached; None when the
        flyover bbox is not trustworthy)."""
        if hasattr(self, "_nav_out_cache"):
            return self._nav_out_cache
        m = None
        try:
            if self._bbox_reliable():
                x0, x1, y0, y1 = self._envelope(slack=NAV_ENV_SLACK_M)
                n = self._nav_N
                idx = np.arange(n)
                xs = (idx - n / 2) * NAV_RES          # column -> x
                ys = (n / 2 - idx) * NAV_RES          # row -> y
                m = ((ys[:, None] < y0) | (ys[:, None] > y1)
                     | (xs[None, :] < x0) | (xs[None, :] > x1))
                print(f"[{self.robot_id}] nav4b planning envelope "
                      f"x {x0:.1f}..{x1:.1f}  y {y0:.1f}..{y1:.1f}")
        except Exception:
            m = None
        self._nav_out_cache = m
        return m

    def _path_blocked(self):''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
