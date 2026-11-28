"""exp08: a failed look at a flyover estimate earns one second look from a
different side. usage: python make_exp08.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("SCAN_NEAR_M         = 3.0",
    "SECOND_LOOK         = True   # exp08: re-scan a missed estimate from another side\n"
    "SECOND_LOOK_R_M     = 1.2    # vantage distance from the estimate\n"
    "SCAN_NEAR_M         = 3.0")

# arrival radius: second-look points are real vantage points
rep("""            arrive_r = (ARRIVE_VIDEO_M if self.target_idx < self._n_video_wps
                        else ARRIVE_SWEEP_M)""",
"""            arrive_r = (ARRIVE_VIDEO_M if self.target_idx < self._n_video_wps
                        else ARRIVE_SWEEP_M)
            if (round(tx, 2), round(ty, 2)) in getattr(self, "_second_pts", ()):
                arrive_r = ARRIVE_SWEEP_M""")

# scan timeout -> maybe queue a second look
rep("""        self._scan_ticks_left -= 1
        if self._scan_ticks_left <= 0:
            self._scanning      = False
            self.target_idx    += 1""",
"""        self._scan_ticks_left -= 1
        if self._scan_ticks_left <= 0:
            self._scanning      = False
            self._queue_second_look()
            self.target_idx    += 1""")

rep("""    def _scan_step(self):""",
'''    def _queue_second_look(self):
        """exp08: a scan at a flyover estimate that saw no victim is usually
        occlusion, not absence (medium exp07 s0: robot2 scanned 1.5 m from v1's
        estimate, saw only a sliver of denim past a cart, and moved on). Queue
        one look from the side of the estimate most different from where we
        stood, on a cell the map believes is free. Once per estimate."""
        if not SECOND_LOOK or self.target_idx >= self._n_video_wps:
            return
        ex, ey = self._my_waypoints[self.target_idx]
        key = (round(ex, 2), round(ey, 2))
        pts = self.__dict__.setdefault("_second_pts", set())
        done = self.__dict__.setdefault("_second_done", set())
        if key in pts or key in done:
            return
        done.add(key)
        here = math.atan2(self.y - ey, self.x - ex)
        best, best_score = None, -1.0
        for k in range(8):
            a = here + math.pi + (k - 4) * math.pi / 4.0
            px, py = ex + SECOND_LOOK_R_M * math.cos(a), ey + SECOND_LOOK_R_M * math.sin(a)
            if self._nav_wall is not None:
                r, c = self._w2c(px, py)
                n = self._nav_N
                if not (0 <= r < n and 0 <= c < n) or self._nav_wall[r, c]:
                    continue
                low = getattr(self, "_nav_low", None)
                if low is not None and low[r, c]:
                    continue
            sep = abs((a - here + math.pi) % (2 * math.pi) - math.pi)
            if sep > best_score:
                best, best_score = (px, py), sep
        if best is None or best_score < math.pi / 2:
            return
        pt = (round(best[0], 2), round(best[1], 2))
        pts.add(pt)
        self._my_waypoints.insert(self.target_idx + 1, pt)
        self._n_video_wps += 1
        print(f"[{self.robot_id}] Second look at ({ex:.1f},{ey:.1f}) "
              f"from ({pt[0]:.1f},{pt[1]:.1f})")

    def _scan_step(self):''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
