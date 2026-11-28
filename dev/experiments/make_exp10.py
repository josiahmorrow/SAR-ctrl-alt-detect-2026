"""exp10: known victims are recognised by where the BODY is, not where the
robot stood. usage: python make_exp10.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new, count=1):
    global src
    n = src.count(old)
    assert n == count, (n, old[:70])
    src = src.replace(old, new)

rep("SECOND_LOOK         = True",
    "KNOWN_BODY_GATE     = True   # exp10: no homing/report at a body already in the map\n"
    "KNOWN_BODY_R_M      = 0.4    # body point this close to a painted victim = known\n"
    "PARTNER_BODY_R_M    = 1.0    # disc painted round a partner's report position\n"
    "SECOND_LOOK         = True")

# victim layer marking inside the body paint loop
rep("""                r, c = self._w2c(self.x + a * ca - l * sa, self.y + a * sa + l * ca)
                self._low_mark(r, r + 1, c, c + 1)""",
"""                r, c = self._w2c(self.x + a * ca - l * sa, self.y + a * sa + l * ca)
                self._low_mark(r, r + 1, c, c + 1)
                self._vic_mark(r, c)""")

rep("""    def _paint_victim_body(self):""",
'''    def _vic_mark(self, r, c):
        """exp10: cells covered by a confirmed victim's body (own or partner)."""
        if not KNOWN_BODY_GATE or self._nav_wall is None:
            return
        if getattr(self, "_nav_vic", None) is None:
            self._nav_vic = np.zeros_like(self._nav_wall)
        n = self._nav_N
        if 0 <= r < n and 0 <= c < n:
            self._nav_vic[r, c] = 1

    def _vic_disc(self, x, y, rad):
        k = int(round(rad / NAV_RES))
        r0, c0 = self._w2c(x, y)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc <= k * k:
                    self._vic_mark(r0 + dr, c0 + dc)

    def _body_known(self, depth, offset):
        """exp10: project the denim blob to the floor (bearing from its image
        column, range from the depth camera) and ask whether that point lies
        on a victim already confirmed. Robot-position checks fail when two
        reports of one victim are made from different sides, or after
        odometry drift between them (small exp07: 3 of 4 False reports were
        repeat reports at found victims)."""
        vic = getattr(self, "_nav_vic", None)
        if not KNOWN_BODY_GATE or vic is None or not math.isfinite(depth):
            return False
        try:
            hfov = self._camera.getFov()
        except Exception:
            hfov = 1.0
        b  = self.theta - offset * hfov / 2.0
        bx = self.x + depth * math.cos(b)
        by = self.y + depth * math.sin(b)
        k  = int(round(KNOWN_BODY_R_M / NAV_RES))
        r0, c0 = self._w2c(bx, by)
        n  = self._nav_N
        r_lo, r_hi = max(0, r0 - k), min(n, r0 + k + 1)
        c_lo, c_hi = max(0, c0 - k), min(n, c0 + k + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            return False
        return bool(vic[r_lo:r_hi, c_lo:c_hi].any())

    def _paint_victim_body(self):''')

# partner's confirmed victim -> disc
rep("""                    if not self._already_reported_nearby(vx, vy):
                        self._reported_positions.append((vx, vy))""",
"""                    self._vic_disc(vx, vy, PARTNER_BODY_R_M)
                    if not self._already_reported_nearby(vx, vy):
                        self._reported_positions.append((vx, vy))""")

# homing gate
rep("""        conf, depth, _ = self._camera_victim_confidence()
        # Require a FINITE denim depth in range""",
"""        conf, depth, _off = self._camera_victim_confidence()
        if conf >= APPROACH_ENTER_CONF and self._body_known(depth, _off):
            self._home_block = "known-body"
            return
        # Require a FINITE denim depth in range""")

# report gate (direct path)
rep("""            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
            # Retire the waypoint ONLY""",
"""            if (not self._already_reported_nearby(self.x, self.y, radius=1.0)
                    and not self._body_known(depth, offset)):
                self.report_victim()
            # Retire the waypoint ONLY""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
