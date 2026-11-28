"""exp18: a body seen END-ON is reported from its flank, not its feet.
usage: make_exp18.py <in> <out>

Geometry: the scored point is the victim's node (pelvis region). Parked
0.75 m from denim seen end-on (blob < 0.45 m wide), the pelvis lies along the
line of sight ~0.35-0.85 m BEYOND the visible surface, i.e. 1.1-1.6 m from the
robot: outside the 1.0 m radius (medium victim3 reported at 1.13-1.20 m in
nav3b M0/M2 and exp08 M1; small exp08 s2 at 1.02 m). Closing the gap
head-on means climbing the legs. Instead step sideways FLANK_SIDE_M onto the
side with more lidar clearance, then drive forward parallel to the body to
abreast its far end, and report there (~0.6 m from the pelvis).
Reuses the reposition machinery (stall, IR body check, timeout -> report).
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "FLANK               = True   # exp18: report end-on bodies from the flank\n"
    "FLANK_END_ON_W_M    = 0.45   # exp18: blob narrower than this = end-on\n"
    "FLANK_SIDE_M        = 0.60   # exp18: lateral offset from the body axis\n"
    "FLANK_ALONG_M       = 0.40   # exp18: beyond the visible surface\n"
    "FLANK_ARRIVE_M      = 0.15   # exp18\n"
    "FLANK_TIMEOUT_S     = 9.0    # exp18\n"
    "SECOND_LOOK         = True")

rep("        self._repositioning   = None\n",
    "        self._repositioning   = None\n"
    "        self._flank_next      = None    # exp18\n"
    "        self._flank_active    = False   # exp18\n"
    "        self._flanked         = False   # exp18\n", n=2)

# helper before _approach_victim
rep("""    def _approach_victim(self):""",
    """    def _flank_cell_free(self, x, y):
        if self._nav_wall is None:
            return True
        r, c = self._w2c(x, y)
        n = self._nav_N
        if not (2 <= r < n - 2 and 2 <= c < n - 2):
            return False
        if self._nav_wall[r - 2:r + 3, c - 2:c + 3].any():
            return False
        low = getattr(self, "_nav_low", None)
        if low is not None and low[r - 1:r + 2, c - 1:c + 2].any():
            return False
        return True

    def _flank_points(self, depth):
        \"\"\"Two-leg flank route, or None if neither side has room.\"\"\"
        fx, fy = math.cos(self.theta), math.sin(self.theta)
        best = None
        for s in (1.0, -1.0):                       # +1 = left of the nose
            lx, ly = -fy * s, fx * s
            room = self._range_at_bearing(s * math.pi / 2.0, half_deg=25.0)
            if room < FLANK_SIDE_M + 0.30:
                continue
            p1 = (self.x + 0.15 * fx + FLANK_SIDE_M * lx,
                  self.y + 0.15 * fy + FLANK_SIDE_M * ly)
            a = depth + FLANK_ALONG_M
            p2 = (self.x + a * fx + FLANK_SIDE_M * lx,
                  self.y + a * fy + FLANK_SIDE_M * ly)
            if not (self._flank_cell_free(*p1) and self._flank_cell_free(*p2)):
                continue
            if best is None or room > best[0]:
                best = (room, p1, p2)
        return None if best is None else (best[1], best[2])

    def _approach_victim(self):""")

# arrival handling in the reposition branch
rep("""            if d <= REPOSITION_STOP_M or now > self._repo_deadline or stalled:
                self.stop()
                self._repositioning = None""",
    """            stop_r = FLANK_ARRIVE_M if self._flank_active else REPOSITION_STOP_M
            if (self._flank_active and self._flank_next is not None
                    and d <= stop_r and now <= self._repo_deadline):
                self._repositioning = self._flank_next      # leg 2
                self._flank_next    = None
                self._repo_prev     = None
                return
            if d <= stop_r or now > self._repo_deadline or stalled:
                self.stop()
                self._repositioning = None
                self._flank_active  = False
                self._flank_next    = None""")

# decision: end-on body -> flank before any CSV reposition
rep("""            est, d_est = self._nearest_estimate()
            if est is not None and REPO_MATCH:""",
    """            if FLANK and not self._flanked:
                _wm = (self._last_vision or {}).get("width_m", float("nan"))
                if math.isfinite(_wm) and _wm < FLANK_END_ON_W_M:
                    _pts = self._flank_points(depth)
                    self._flanked = True
                    if _pts is not None:
                        self._flank_active    = True
                        self._repositioning   = _pts[0]
                        self._flank_next      = _pts[1]
                        self._repo_deadline   = now + FLANK_TIMEOUT_S
                        self._repo_prev       = None
                        self._home_start_time = now
                        print(f"[{self.robot_id}] End-on body (w={_wm:.2f} m) - "
                              f"flanking via ({_pts[0][0]:.2f},{_pts[0][1]:.2f}) to "
                              f"({_pts[1][0]:.2f},{_pts[1][1]:.2f})")
                        return
            est, d_est = self._nearest_estimate()
            if est is not None and REPO_MATCH:""")

# reset per homing episode
rep("""    def _end_homing(self, advance=False):""",
    """    def _end_homing(self, advance=False):
        self._flanked = False
        self._flank_active = False
        self._flank_next = None
        self._end_homing_inner(advance)

    def _end_homing_inner(self, advance=False):""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
