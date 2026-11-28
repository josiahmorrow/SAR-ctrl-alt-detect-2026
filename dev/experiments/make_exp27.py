"""exp27: keep, and licence, a flyover estimate that a report disclaimed.
usage: make_exp27.py <in> <out>

snapS6 / missed.py on exp24-26 small runs: robot2 homes toward the v1
estimate (2.94,-3.12), meets the v2 body first, and REPO_MATCH correctly
prints "Estimate (2.94,-3.12) is not this body's - reporting from here".
The v2 report is true, but then:
  * the homing branch advances the itinerary (robot within 2.5 m of the
    target) so the v1 waypoint is dropped although the report disclaimed it;
  * every later sighting of v1 is inside HOME_BLOCK_R_M (1.5 m) of the v2
    report pose, so homing is refused ("near-prior-report", 147 frames);
  * a report parked at v1 would sit < 1.0 m from the v2 report pose and be
    swallowed by the report-time dedup.
The disclaimer is positive geometric evidence that a second body exists
near the first. exp27 records (estimate, report pose) pairs the report
disclaimed, keeps that waypoint, and relaxes the two proximity gates only
when the body in view projects (same pose + (depth+0.3) projection as
REPO_MATCH) within REPO_MATCH_M of a still-open disclaimed estimate and no
other report is near. The exact-double guard (0.5 m) always stays.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


# --- constant --------------------------------------------------------------
rep("HOME_BLOCK_R_M      = 1.5\n",
    "HOME_BLOCK_R_M      = 1.5\n"
    "TWO_BODY            = True   # exp27: licence an estimate a report disclaimed\n")

# --- state -----------------------------------------------------------------
rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._disclaimed       = []      # exp27: (ex, ey, rx, ry)\n"
    "        self._licence          = None    # exp27: estimate licensed now\n")

# --- helpers ---------------------------------------------------------------
rep("    def _broadcast_victim(self, x, y):\n",
    '''    def _disc_open(self, ex, ey, rx, ry):
        """exp27: a disclaimed estimate stays open until some OTHER report
        lands within 1.0 m of it."""
        return not any(math.hypot(ex - px, ey - py) < 1.0
                       and math.hypot(px - rx, py - ry) > 0.05
                       for px, py in self._reported_positions)

    def _two_body_licence(self):
        """exp27: the estimate the body in view belongs to, if that estimate
        was disclaimed by a nearby report and is still open, and no other
        report is within HOME_BLOCK_R_M of the robot. None otherwise."""
        if not (TWO_BODY and self._disclaimed):
            return None
        conf, depth, _ = self._camera_victim_confidence()
        if conf < 0.15 or not math.isfinite(depth) or depth > 3.0:
            return None
        bx = self.x + (depth + 0.3) * math.cos(self.theta)
        by = self.y + (depth + 0.3) * math.sin(self.theta)
        for (ex, ey, rx, ry) in self._disclaimed:
            if math.hypot(bx - ex, by - ey) > REPO_MATCH_M:
                continue
            if not self._disc_open(ex, ey, rx, ry):
                continue
            if any(math.hypot(self.x - px, self.y - py) < HOME_BLOCK_R_M
                   and math.hypot(px - rx, py - ry) > 0.05
                   for px, py in self._reported_positions):
                continue
            return (ex, ey)
        return None

    def _wp_disclaimed(self, tx, ty):
        """exp27: True if (tx, ty) is an open disclaimed estimate."""
        return TWO_BODY and any(
            math.hypot(tx - ex, ty - ey) < 0.3 and self._disc_open(ex, ey, rx, ry)
            for (ex, ey, rx, ry) in self._disclaimed)

    def _dup_blocked(self):
        """Report-time dedup (1.0 m). exp27: under a two-body licence only an
        exact double (0.5 m) blocks."""
        if not self._already_reported_nearby(self.x, self.y, radius=1.0):
            return False
        if TWO_BODY and self._licence is not None and \\
           not self._already_reported_nearby(self.x, self.y, radius=0.5):
            return False
        return True

    def _exp27_after_report(self, n0, disc):
        """exp27: book-keeping after a report attempt."""
        if len(self._reported_positions) <= n0:
            return
        rx, ry = self._reported_positions[-1]
        if self._licence is not None:
            lx, ly = self._licence
            print(f"[{self.robot_id}] exp27 two-body report for estimate "
                  f"({lx:.2f},{ly:.2f})")
        if disc is not None:
            self._disclaimed.append((disc[0], disc[1], rx, ry))
            print(f"[{self.robot_id}] exp27 keeping disclaimed estimate "
                  f"({disc[0]:.2f},{disc[1]:.2f})")

    def _broadcast_victim(self, x, y):
''')

# --- homing entry: licence past near-prior-report -------------------------
rep('''        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
''', '''        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M) and \\
           self._two_body_licence() is None:
            self._home_block = "near-prior-report"
            return
''')

# --- reposition-path report ------------------------------------------------
rep('''                if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                    self.report_victim()
''', '''                _n0 = len(self._reported_positions)
                if not self._dup_blocked():
                    self.report_victim()
                if TWO_BODY:
                    self._exp27_after_report(_n0, None)
''')

# --- main report branch: licence decided once the streak completes ---------
rep('''            self.stop()
            # Marker-offset countermeasure: the CSV estimate is a better proxy
''', '''            self.stop()
            _disc = None
            self._licence = None
            if TWO_BODY and self._already_reported_nearby(self.x, self.y,
                                                          radius=1.0):
                self._licence = self._two_body_licence()
            # Marker-offset countermeasure: the CSV estimate is a better proxy
''')
rep('''                          f"is not this body's - reporting from here")
                    est = None
''', '''                          f"is not this body's - reporting from here")
                    _disc = est
                    est = None
''')
rep('''            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
''', '''            _n0 = len(self._reported_positions)
            if not self._dup_blocked():
                self.report_victim()
            if TWO_BODY:
                self._exp27_after_report(_n0, _disc)
''')
rep('''                tx, ty   = self._my_waypoints[self.target_idx]
                at_target = math.hypot(self.x - tx, self.y - ty) <= 2.5
''', '''                tx, ty   = self._my_waypoints[self.target_idx]
                at_target = math.hypot(self.x - tx, self.y - ty) <= 2.5
                if self._wp_disclaimed(tx, ty):
                    at_target = False          # exp27: report disclaimed it
''')

# --- clear licence when homing ends ----------------------------------------
rep('''        self._report_streak   = 0
        self._repositioning   = None
        self.stop()
        if advance and self.target_idx < len(self._my_waypoints):
''', '''        self._report_streak   = 0
        self._repositioning   = None
        self._licence         = None   # exp27
        self.stop()
        if advance and self.target_idx < len(self._my_waypoints):
''')

# --- waypoint retirement spares an open disclaimed estimate ----------------
rep('''            if any(math.hypot(tx - rx, ty - ry) < WP_RETIRE_M
                   for rx, ry in self._reported_positions):
                print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) "
                      f"retired''', '''            if any(math.hypot(tx - rx, ty - ry) < WP_RETIRE_M
                   for rx, ry in self._reported_positions) and \\
               not self._wp_disclaimed(tx, ty):
                print(f"[{self.robot_id}] Waypoint ({tx:.1f},{ty:.1f}) "
                      f"retired''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
