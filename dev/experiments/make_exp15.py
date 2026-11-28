"""exp15: de-duplicate own reports by WHERE THE VICTIM IS, not where the robot
parked. usage: make_exp15.py <in> <out>

Geometry: two victims are distinct bodies; two parking spots are not. The old
gate blocked homing whenever the robot stood within 1.5 m of ANY earlier
parking spot, so a second victim 1.4 m from the first (small v1/v2) could never
be homed on by the robot that found the first. Own reports now store a body
estimate (pose + camera bearing + denim depth + centroid allowance by lie);
a candidate is a duplicate only if its body estimate is within BODY_BLOCK_R_M
of an own body. Partner reports stay on the old wide robot-position gate,
because the two robots' odometry frames drift apart by up to ~1.5 m.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


# constants
rep("SECOND_LOOK         = True",
    "BODY_DEDUP          = True   # exp15: own-report dedup on body estimate\n"
    "BODY_BLOCK_R_M      = 1.0    # exp15: body-to-body duplicate radius\n"
    "SECOND_LOOK         = True")

# state
rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._own_bodies       = []      # exp15\n"
    "        self._partner_reports  = []      # exp15\n"
    "        self._cand_body        = None    # exp15\n")

# keep x_offset in the candidate record
rep('                             "width_m": width_m}\n        return base_conf, min_depth, x_offset',
    '                             "width_m": width_m, "x_offset": x_offset}\n'
    '        return base_conf, min_depth, x_offset')

# partner reports tracked separately
rep("""                    if not self._already_reported_nearby(vx, vy):
                        self._reported_positions.append((vx, vy))
""", """                    self._partner_reports.append((vx, vy))
                    if not self._already_reported_nearby(vx, vy):
                        self._reported_positions.append((vx, vy))
""")

# own report stores the body estimate
rep("""        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)
""", """        self._reported_positions.append((self.x, self.y))
        if self._cand_body is not None:
            self._own_bodies.append(self._cand_body)
        else:
            self._own_bodies.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)
""")

# helpers, placed before _broadcast_victim
rep("""    def _broadcast_victim(self, x, y):""",
    """    def _body_estimate(self, depth, x_offset):
        \"\"\"exp15: world position of the observed victim's scored centroid:
        camera bearing from the denim blob's centroid, range = median denim
        depth + an allowance for how the body lies (broadside: ~0.2 m behind
        the visible surface; end-on: ~0.6 m).\"\"\"
        if not (math.isfinite(depth) and depth < 4.0):
            return None
        try:
            hf = float(self._camera.getFov())
        except Exception:
            hf = 1.0
        bearing = -math.atan(x_offset * math.tan(hf * 0.5))
        wm = (self._last_vision or {}).get("width_m", float("nan"))
        extra = 0.2 if (math.isfinite(wm) and wm > 0.9) else 0.6
        r = depth + extra
        a = self.theta + bearing
        return (self.x + r * math.cos(a), self.y + r * math.sin(a))

    def _dup_of_own(self, body):
        return body is not None and any(
            math.hypot(body[0] - bx, body[1] - by) < BODY_BLOCK_R_M
            for bx, by in self._own_bodies)

    def _near_partner_report(self, radius):
        return any(math.hypot(self.x - px, self.y - py) < radius
                   for px, py in self._partner_reports)

    def _is_duplicate_report(self):
        \"\"\"exp15 replacement for the robot-position 1.0 m report gate.\"\"\"
        if self._near_partner_report(1.0):
            return True
        if self._cand_body is None:
            return any(math.hypot(self.x - bx, self.y - by) < 1.0
                       for bx, by in self._own_bodies)
        return self._dup_of_own(self._cand_body)

    def _broadcast_victim(self, x, y):""")

# homing gate
rep("""        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
        conf, depth, _ = self._camera_victim_confidence()
""", """        if not BODY_DEDUP and self._already_reported_nearby(
                self.x, self.y, radius=HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
        if BODY_DEDUP and self._near_partner_report(HOME_BLOCK_R_M):
            self._home_block = "near-prior-report"
            return
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
        conf, depth, _off = self._camera_victim_confidence()
        if BODY_DEDUP:
            body = self._body_estimate(depth, _off)
            if self._dup_of_own(body):
                self._home_block = "near-prior-report"
                return
            if body is None and any(
                    math.hypot(self.x - bx, self.y - by) < HOME_BLOCK_R_M
                    for bx, by in self._own_bodies):
                self._home_block = "near-prior-report"
                return
            self._cand_body = body
""")

# refresh the body estimate on every homing detection
rep("""        conf, depth, offset = self._camera_victim_confidence()

        if self._home_start_time is not None and \\""",
    """        conf, depth, offset = self._camera_victim_confidence()
        if BODY_DEDUP:
            _b = self._body_estimate(depth, offset)
            if _b is not None:
                self._cand_body = _b

        if self._home_start_time is not None and \\""")

# the two report gates
rep("""                if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                    self.report_victim()""",
    """                if not (self._is_duplicate_report() if BODY_DEDUP else
                        self._already_reported_nearby(self.x, self.y, radius=1.0)):
                    self.report_victim()""")
rep("""            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()""",
    """            if not (self._is_duplicate_report() if BODY_DEDUP else
                    self._already_reported_nearby(self.x, self.y, radius=1.0)):
                self.report_victim()""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
