"""nav2: look where the camera has not looked. usage: make_nav2.py <in> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("NAV_LOGODDS       = True",
    "NAV_LOGODDS       = True\n"
    "# nav2: lidar coverage is not camera coverage. Robots on nav1 reached 99% of\n"
    "# small's floor by lidar and still drove past victims at ~1 m without a\n"
    "# single candidate frame: the camera faced along the route. Keep a grid of\n"
    "# cells the camera has actually seen (FOV, 3 m, cut at the lidar return)\n"
    "# and stop to turn on the spot when most free floor nearby is unseen.\n"
    "LOOK_AROUND       = True\n"
    "LOOK_RANGE_M      = 3.0    # camera reach credited along a ray\n"
    "LOOK_R_M          = 2.5    # neighbourhood judged for unseen floor\n"
    "LOOK_TRIGGER      = 0.5    # unseen fraction that triggers a look-around\n"
    "LOOK_S            = 6.5    # one full turn on the spot\n"
    "LOOK_GAP_S        = 10.0   # minimum time between look-arounds")

rep("""            if not self._homing:
                self._maybe_start_homing()
            if self._homing:
                self._approach_victim()
            else:
                self.navigate_step()""",
"""            if not self._homing:
                self._maybe_start_homing()
            if self._homing:
                self._approach_victim()
            elif not self._look_step():
                self.navigate_step()""")

rep("""    def _read_tilt(self):""",
'''    def _update_look(self, now):
        """nav2: credit the cells inside the camera's field of view, up to
        LOOK_RANGE_M or the lidar return in that direction, whichever is
        nearer (walls hide what is behind them)."""
        if self._nav_wall is None or not self._lidar:
            return
        if now < getattr(self, "_look_next_t", 0.0):
            return
        self._look_next_t = now + 0.5
        if getattr(self, "_nav_look", None) is None:
            self._nav_look = np.zeros_like(self._nav_wall)
        try:
            ranges = self._lidar.getRangeImage()
            hfov = self._camera.getFov()
        except Exception:
            return
        n, N = len(ranges), self._nav_N
        if n == 0:
            return
        for k in range(13):
            b = -hfov / 2.0 + hfov * k / 12.0
            i = int(round((math.pi - b) * n / (2.0 * math.pi))) % n
            rr_ = ranges[i]
            rmax = min(LOOK_RANGE_M, rr_ if (rr_ == rr_ and rr_ < 1e5) else LOOK_RANGE_M)
            ca, sa = math.cos(self.theta + b), math.sin(self.theta + b)
            d = 0.2
            while d <= rmax:
                r, c = self._w2c(self.x + d * ca, self.y + d * sa)
                if 0 <= r < N and 0 <= c < N:
                    self._nav_look[r, c] = 1
                d += 0.07

    def _look_step(self):
        """nav2: returns True while a look-around owns the wheels."""
        if not LOOK_AROUND or self._nav_wall is None:
            return False
        now = self.robot.getTime()
        self._update_look(now)
        if now < getattr(self, "_look_until", 0.0):
            self.set_speeds(-SCAN_TURN_SPEED, SCAN_TURN_SPEED)
            return True
        if (self._scanning or now < getattr(self, "_look_check_t", 0.0)
                or now - getattr(self, "_look_last", -1e9) < LOOK_GAP_S
                or getattr(self, "_nav_look", None) is None):
            return False
        self._look_check_t = now + 1.0
        k = int(round(LOOK_R_M / NAV_RES))
        r0, c0 = self._w2c(self.x, self.y)
        N = self._nav_N
        rs, re_ = max(0, r0 - k), min(N, r0 + k + 1)
        cs, ce = max(0, c0 - k), min(N, c0 + k + 1)
        if rs >= re_ or cs >= ce:
            return False
        free = (self._nav_seen[rs:re_, cs:ce] > 0) & (self._nav_wall[rs:re_, cs:ce] == 0)
        nf = int(free.sum())
        if nf < 50:
            return False
        unseen = int((free & (self._nav_look[rs:re_, cs:ce] == 0)).sum()) / nf
        if unseen < LOOK_TRIGGER:
            return False
        self._look_until = now + LOOK_S
        self._look_last = now
        if self._wp_start_time is not None:           # a look is not lost time
            self._wp_start_time += LOOK_S
            self._wp_best_t     += LOOK_S
        self._look_count = getattr(self, "_look_count", 0) + 1
        if self._look_count <= 3 or self._look_count % 10 == 0:
            print(f"[{self.robot_id}] Look-around #{self._look_count} "
                  f"({unseen:.0%} of nearby floor unseen by camera)")
        self.stop()
        return True

    def _read_tilt(self):''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
