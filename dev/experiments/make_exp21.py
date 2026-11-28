"""exp21 (on exp19): a second victim next to a reported one may be homed and
reported when an independent flyover estimate vouches for it.
usage: make_exp21.py <in> <out>

The near-prior-report gate (1.5 m homing, 1.0 m report) exists so a robot
does not re-report a victim already found. It also hides a SECOND victim
lying within 1.5 m of the first (small v1/v2 are 1.4 m apart). exp19 made
this visible: reporting from the body, robot2 reports v2 and then is
blocked from v1 for the rest of the run (exp19 small s2: 1/4).
Evidence that the candidate is a different person: the body in view (pose +
depth + 0.3 m along the camera axis) is within EST_MATCH_M of a flyover
estimate that no own report has claimed. On each own report, the estimate
nearest the reported body (within EST_MATCH_M) is marked claimed.
Partner reports keep the old robot-position gates.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "EST_BACKED          = True   # exp21: estimate-backed second victim\n"
    "EST_MATCH_M         = 1.2    # exp21: body-to-estimate match radius\n"
    "SECOND_LOOK         = True")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._own_reports      = []      # exp21: own report positions\n"
    "        self._claimed_est      = []      # exp21: estimates claimed by own reports\n"
    "        self._est_backed       = None    # exp21: estimate vouching for this homing\n")

rep("""    def _already_reported_nearby(self, x, y, radius=0.5):""",
    """    def _body_ahead(self, depth):
        return (self.x + (depth + 0.3) * math.cos(self.theta),
                self.y + (depth + 0.3) * math.sin(self.theta))

    def _est_for_body(self, bx, by):
        best, bd = None, EST_MATCH_M
        for e in getattr(self, "_video_estimates", []) or []:
            d = math.hypot(e[0] - bx, e[1] - by)
            if d < bd:
                best, bd = e, d
        return best

    def _unclaimed_est_for_body(self, depth):
        if not (math.isfinite(depth) and depth < 4.0):
            return None
        e = self._est_for_body(*self._body_ahead(depth))
        if e is None or e in self._claimed_est:
            return None
        # A partner-reported victim near this estimate is not ours to re-find.
        if any(math.hypot(e[0] - px, e[1] - py) < 1.0
               for px, py in self._reported_positions
               if (px, py) not in self._own_reports):
            return None
        return e

    def _already_reported_nearby(self, x, y, radius=0.5):""")

# record own reports and claim the estimate for the reported body
rep("""        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)
""", """        self._reported_positions.append((self.x, self.y))
        self._own_reports.append((self.x, self.y))
        if EST_BACKED:
            _d = (self._last_vision or {}).get("depth", float("inf"))
            _e = self._est_backed
            if _e is None and math.isfinite(_d) and _d < 2.0:
                _e = self._est_for_body(*self._body_ahead(_d))
            if _e is None:
                _e = self._est_for_body(self.x, self.y)
            if _e is not None and _e not in self._claimed_est:
                self._claimed_est.append(_e)
        self._broadcast_victim(self.x, self.y)
""")

# homing gate: allow when an unclaimed estimate vouches for the body in view
rep("""        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return""",
    """        self._est_backed = None
        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            _e = None
            if EST_BACKED:
                _c, _dp, _o = self._camera_victim_confidence()
                if _c >= APPROACH_ENTER_CONF:
                    _e = self._unclaimed_est_for_body(_dp)
            if _e is None:
                self._home_block = "near-prior-report"
                return
            self._est_backed = _e
            print(f"[{self.robot_id}] Second victim near a prior report - "
                  f"estimate ({_e[0]:.2f},{_e[1]:.2f}) unclaimed, homing")""")

# report gates: an estimate-backed homing may report near a prior report
rep("""                if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                    self.report_victim()""",
    """                if (self._est_backed is not None
                        or not self._already_reported_nearby(self.x, self.y, radius=1.0)):
                    self.report_victim()""")
rep("""            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()""",
    """            if (self._est_backed is not None
                    or not self._already_reported_nearby(self.x, self.y, radius=1.0)):
                self.report_victim()""")

# clear per homing episode
rep("""        self._homing          = False
        self._home_start_time = None
        self._home_lost_ticks = 0
        self._report_streak   = 0
        self._repositioning   = None
        self.stop()""",
    """        self._homing          = False
        self._home_start_time = None
        self._home_lost_ticks = 0
        self._report_streak   = 0
        self._repositioning   = None
        self._est_backed      = None
        self.stop()""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
