"""dr4: reporting rules re-based on the flyover estimates (one estimate, one
victim). usage: make_dr4.py <in> <out> [--arrival]

With flyover_dr the estimates are within about 1 m of the victims, so they
can carry the bookkeeping that robot positions carried before:
  * every report (own or partner's) CONSUMES the nearest unconsumed
    estimate within 1.5 m (own: of the reported body's projected position;
    partner: of the partner's report pose);
  * inside the old no-report zones (1.5 m homing block, 1.0 m report
    dedup) a candidate is still allowed when its projected position lies
    within 1.0 m of an UNCONSUMED estimate and nearer to it than to any
    consumed one. The 0.5 m exact-double guard stays.
cln small: v1 (1.4 m from v2) was seen 21-39 times per run and refused
every time as "near-prior-report".

--arrival (dr5): after a full scan at a waypoint with no candidate, if the
robot stands within 0.7 m of an unconsumed estimate, report on position.
cln small: robot2 sat within 1.0 m of v4 for 100+ s with no denim in view.
Riskier: the extra estimates (bedding, forklift) will draw false reports.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
ARRIVAL = "--arrival" in sys.argv


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("HOME_BLOCK_R_M      = 1.5\n",
    "HOME_BLOCK_R_M      = 1.5\n"
    "EST_BOOK            = True   # dr4: estimate-based report bookkeeping\n"
    "EST_CONSUME_R_M     = 1.5    # dr4\n"
    "EST_LICENCE_R_M     = 1.0    # dr4: the scoring radius\n"
    "ARRIVAL_REPORT      = %s  # dr5\n"
    "ARRIVAL_R_M         = 0.7    # dr5\n" % ("True " if ARRIVAL else "False"))

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._consumed         = []      # dr4: estimates already reported\n")

rep("    def _broadcast_victim(self, x, y):\n", '''    def _consume_estimate(self, x, y):
        """dr4: mark the nearest unconsumed flyover estimate as reported."""
        best, bd = None, EST_CONSUME_R_M
        for e in getattr(self, "_video_estimates", []):
            if e in self._consumed:
                continue
            d = math.hypot(e[0] - x, e[1] - y)
            if d < bd:
                best, bd = e, d
        if best is not None:
            self._consumed.append(best)
            print(f"[{self.robot_id}] dr4 estimate ({best[0]:.2f},{best[1]:.2f}) "
                  f"consumed by report near ({x:.2f},{y:.2f})")

    def _body_point(self):
        """dr4: ground position of the candidate in view, or None."""
        v = self._last_vision or {}
        d = v.get("depth", float("inf"))
        if v.get("why") != "candidate" or not math.isfinite(d):
            return None
        try:
            hf = float(self._camera.getFov())
        except Exception:
            hf = 1.0
        th = self.theta - math.atan(v.get("x_offset", 0.0) * math.tan(hf * 0.5))
        return (self.x + (d + 0.3) * math.cos(th), self.y + (d + 0.3) * math.sin(th))

    def _est_licence(self):
        """dr4: the unconsumed estimate the candidate in view belongs to, if it
        is within EST_LICENCE_R_M of it and nearer to it than to any consumed
        estimate. None otherwise."""
        if not EST_BOOK:
            return None
        bp = self._body_point()
        if bp is None:
            return None
        best, bd = None, EST_LICENCE_R_M
        for e in getattr(self, "_video_estimates", []):
            if e in self._consumed:
                continue
            d = math.hypot(e[0] - bp[0], e[1] - bp[1])
            if d < bd:
                best, bd = e, d
        if best is None:
            return None
        if any(math.hypot(c[0] - bp[0], c[1] - bp[1]) <= bd for c in self._consumed):
            return None
        return best

    def _report_blocked(self):
        """Report-time dedup (1.0 m), with the dr4 licence: only an exact
        double (0.5 m) blocks a candidate that belongs to an unconsumed
        estimate."""
        if not self._already_reported_nearby(self.x, self.y, radius=1.0):
            return False
        if self._est_licence() is not None and \\
           not self._already_reported_nearby(self.x, self.y, radius=0.5):
            print(f"[{self.robot_id}] dr4 licensed report near a prior one")
            return False
        return True

    def _broadcast_victim(self, x, y):
''')

# own report consumes an estimate
rep("        self._reported_positions.append((self.x, self.y))\n"
    "        self._broadcast_victim(self.x, self.y)\n",
    "        self._reported_positions.append((self.x, self.y))\n"
    "        if EST_BOOK:\n"
    "            _bp = self._body_point() or (self.x, self.y)\n"
    "            self._consume_estimate(_bp[0], _bp[1])\n"
    "        self._broadcast_victim(self.x, self.y)\n")

# partner report consumes an estimate
rep("                        self._reported_positions.append((vx, vy))\n",
    "                        self._reported_positions.append((vx, vy))\n"
    "                        if EST_BOOK:\n"
    "                            self._consume_estimate(vx, vy)\n")

# homing entry
rep('''        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
''', '''        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M) and \\
           self._est_licence() is None:
            self._home_block = "near-prior-report"
            return
''')

# report-time dedup (reposition branch and main branch)
rep('''                if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                    self.report_victim()
''', '''                if not self._report_blocked():
                    self.report_victim()
''')
rep('''            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
''', '''            if not self._report_blocked():
                self.report_victim()
''')

# dr5: arrival report after an empty scan
rep('''        if self._scan_ticks_left <= 0:
            self._scanning      = False
''', '''        if self._scan_ticks_left <= 0:
            self._scanning      = False
            if ARRIVAL_REPORT and EST_BOOK:
                _ne = [e for e in getattr(self, "_video_estimates", [])
                       if e not in self._consumed
                       and math.hypot(e[0] - self.x, e[1] - self.y) <= ARRIVAL_R_M]
                if _ne and not self._already_reported_nearby(self.x, self.y,
                                                             radius=0.5):
                    print(f"[{self.robot_id}] dr5 arrival report at estimate "
                          f"({_ne[0][0]:.2f},{_ne[0][1]:.2f})")
                    self._last_vision = {"why": "arrival"}
                    self.report_victim()
''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
