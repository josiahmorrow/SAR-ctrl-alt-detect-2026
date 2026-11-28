"""co1: target claims between the two robots.
usage: make_co1.py <in> <out>

Observed at 180 s: both robots end up at the same victim and block each
other (b180 medium M0: both within 0.7-0.8 m of v2, each with 20+ candidate
frames, each refused by the partner false-positive zone), or one adopts the
estimate the other is still working.

The 1 Hz 'pos' message (channel 73) already carries the sender's current
flyover target (tx, ty). co1 adds, in the same message:
  hx, hy  ground position of the body the sender is HOMING on
  cx, cy  ground position of a solid candidate the sender sees (not homing)
and uses them, while fresh (CLAIM_FRESH_S):
  1. Drive claim. If my current flyover waypoint is within CLAIM_R_M of the
     partner's target or homing point, the robot that is clearly nearer
     keeps it (ties: robot1); the other moves the waypoint to the end of its
     flyover leg, once per waypoint. If it is the last one in the leg it is
     only dropped when the partner is already homing there.
  2. Homing claim. I do not start homing on a candidate whose ground
     position is within CLAIM_R_M of the body the partner is homing on.
  3. Shared-victim deadlock (CO1_FP_RELIEF). Inside the PARTNER
     false-positive zone (never the origin-mat zone) one robot may proceed
     if the candidate is not at the partner's own position and the partner
     has no better claim: partner homing there -> no; partner sees the same
     candidate -> robot1 only.
Nothing changes while the partner is silent (no fresh 'pos' message).
All new fields are optional; an unpatched partner ignores them.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


# --- flags -------------------------------------------------------------
rep("HOME_BLOCK_R_M      = 1.5\n",
    "HOME_BLOCK_R_M      = 1.5\n"
    "CO1                 = True   # co1: target claims between robots\n"
    "CO1_FP_RELIEF       = True   # co1: break the shared-victim deadlock\n"
    "CLAIM_R_M           = 1.0    # co1: same target / same body\n"
    "CLAIM_FRESH_S       = 3.0    # co1: partner info older than this is ignored\n"
    "CLAIM_MARGIN_M      = 0.5    # co1: 'clearly nearer'\n")

# --- state -------------------------------------------------------------
rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._co1_pos_t        = -1e9    # co1: time of last partner 'pos'\n"
    "        self._co1_p_home       = None    # co1: (x, y, t) partner homing body\n"
    "        self._co1_p_cand       = None    # co1: (x, y, t) partner candidate\n"
    "        self._co1_yielded      = set()   # co1: waypoints yielded once\n")

# --- helpers (placed before _broadcast_victim) --------------------------
rep("    def _broadcast_victim(self, x, y):\n", '''    def _co1_body_point(self):
        """co1: ground position of the candidate in view, or None."""
        v = self._last_vision or {}
        d = v.get("depth", float("inf"))
        if v.get("why") != "candidate" or not isinstance(d, (int, float)) \\
           or not math.isfinite(d):
            return None
        try:
            hf = float(self._camera.getFov())
        except Exception:
            hf = 1.0
        xo = v.get("x_offset", 0.0) or 0.0
        th = self.theta - math.atan(xo * math.tan(hf * 0.5))
        return (self.x + (d + 0.3) * math.cos(th),
                self.y + (d + 0.3) * math.sin(th))

    def _co1_fresh(self, t):
        return (self.robot.getTime() - t) <= CLAIM_FRESH_S

    def _co1_partner_homing_at(self, x, y):
        """co1: True if the partner is (freshly) homing on a body at (x, y)."""
        h = self._co1_p_home
        return (h is not None and self._co1_fresh(h[2])
                and math.hypot(h[0] - x, h[1] - y) < CLAIM_R_M)

    def _co1_should_yield(self, tx, ty):
        """co1 rule 1: give way on a flyover waypoint the partner has claimed.
        Returns True when the itinerary was changed (caller returns)."""
        if not CO1 or self.target_idx >= self._n_video_wps:
            return False
        if not self._co1_fresh(self._co1_pos_t) or self._partner_x is None:
            return False
        key = (round(tx, 2), round(ty, 2))
        homing = self._co1_partner_homing_at(tx, ty)
        claimed = homing
        pt = getattr(self, "_partner_target", None)
        if not claimed and pt is not None:
            claimed = math.hypot(pt[0] - tx, pt[1] - ty) < CLAIM_R_M
        if not claimed:
            return False
        if not homing:
            my_d = math.hypot(self.x - tx, self.y - ty)
            p_d = math.hypot(self._partner_x - tx, self._partner_y - ty)
            if my_d + CLAIM_MARGIN_M < p_d:
                return False                      # I am clearly nearer
            if abs(my_d - p_d) <= CLAIM_MARGIN_M and self.robot_id == "robot1":
                return False                      # tie: robot1 keeps it
        last = self.target_idx >= self._n_video_wps - 1
        if last:
            if not homing:
                return False                      # nothing else to do first
            print(f"[{self.robot_id}] co1 waypoint ({tx:.1f},{ty:.1f}) left to "
                  f"the partner (homing there)")
            self.target_idx += 1
        else:
            if key in self._co1_yielded:
                return False                      # yielded once already
            self._co1_yielded.add(key)
            wp = self._my_waypoints.pop(self.target_idx)
            self._my_waypoints.insert(self._n_video_wps - 1, wp)
            print(f"[{self.robot_id}] co1 waypoint ({tx:.1f},{ty:.1f}) deferred - "
                  f"partner has it")
        self._wp_start_time = None
        self._path = None
        self._scanning = False
        return True

    def _co1_partner_claims_candidate(self):
        """co1 rule 2: the candidate in view is the body the partner is
        homing on."""
        if not CO1:
            return False
        bp = self._co1_body_point()
        return bp is not None and self._co1_partner_homing_at(bp[0], bp[1])

    def _co1_fp_relief(self):
        """co1 rule 3: may this robot ignore the PARTNER false-positive zone
        for the candidate in view?"""
        if not (CO1 and CO1_FP_RELIEF):
            return False
        if math.hypot(self.x, self.y) < 3.0:
            return False                          # origin-mat zone stays
        if self._partner_x is None or not self._co1_fresh(self._co1_pos_t):
            return False
        bp = self._co1_body_point()
        if bp is None:
            return False
        if math.hypot(bp[0] - self._partner_x, bp[1] - self._partner_y) < 0.8:
            return False                          # that may BE the partner
        if self._co1_partner_homing_at(bp[0], bp[1]):
            return False
        c = self._co1_p_cand
        if c is not None and self._co1_fresh(c[2]) \\
           and math.hypot(c[0] - bp[0], c[1] - bp[1]) < CLAIM_R_M \\
           and self.robot_id != "robot1":
            return False                          # both see it: robot1 goes
        return True

    def _broadcast_victim(self, x, y):
''')

# --- send: homing / candidate point in the 'pos' message -----------------
rep('        msg = {"type": "pos", "x": self.x, "y": self.y}\n',
    '        msg = {"type": "pos", "x": self.x, "y": self.y}\n'
    '        if CO1:\n'
    '            _b = self._co1_body_point()\n'
    '            if _b is not None:\n'
    '                _k = ("hx", "hy") if self._homing else ("cx", "cy")\n'
    '                msg[_k[0]], msg[_k[1]] = round(_b[0], 2), round(_b[1], 2)\n')

# --- receive ---------------------------------------------------------------
rep('                    self._partner_y = float(msg["y"])\n',
    '                    self._partner_y = float(msg["y"])\n'
    '                    if CO1:\n'
    '                        _now = self.robot.getTime()\n'
    '                        self._co1_pos_t = _now\n'
    '                        try:\n'
    '                            if "hx" in msg and "hy" in msg:\n'
    '                                self._co1_p_home = (float(msg["hx"]),\n'
    '                                                    float(msg["hy"]), _now)\n'
    '                            if "cx" in msg and "cy" in msg:\n'
    '                                self._co1_p_cand = (float(msg["cx"]),\n'
    '                                                    float(msg["cy"]), _now)\n'
    '                        except (TypeError, ValueError):\n'
    '                            pass\n')

# --- rule 3 + rule 2 at homing entry ---------------------------------------
rep('''        if self._victim_false_positive_zone():
            self._home_block = "fp-zone"
            return
''', '''        if self._victim_false_positive_zone() and not self._co1_fp_relief():
            self._home_block = "fp-zone"
            return
''')
rep('''        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''', '''        if self._co1_partner_claims_candidate():
            self._home_block = "partner-claim"
            return
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''')

# --- rule 1 in the itinerary -----------------------------------------------
rep('''            tx, ty = self._my_waypoints[self.target_idx]

            if self._scanning:
                self._scan_step()
                return
''', '''            tx, ty = self._my_waypoints[self.target_idx]

            if self._scanning:
                self._scan_step()
                return

            if self._co1_should_yield(tx, ty):
                return
''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
