"""exp11: a victim is identified by where its body is, not where the robot
stood when reporting it. usage: python make_exp11.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("SECOND_LOOK         = True",
    "BODY_ID             = True   # exp11: known-victim checks use body points\n"
    "BODY_SAME_R_M       = 0.7    # two denim floor points this close = same victim\n"
    "ROBOT_SAME_R_M      = 0.5    # fallback when no body point is available\n"
    "SECOND_LOOK         = True")

# A. keep the blob bearing in the candidate record
rep('''                             "ratio": ratio, "med_h": med_h, "low_h": low_h,
                             "width_m": width_m}''',
'''                             "ratio": ratio, "med_h": med_h, "low_h": low_h,
                             "width_m": width_m, "offset": x_offset}''')

# helpers before _already_reported_nearby
rep("""    def _already_reported_nearby(self, x, y, radius=0.5):""",
'''    def _body_point(self, depth, offset):
        """exp11: floor point of the denim blob: bearing from its image column,
        range from the depth camera. None without a finite range."""
        if not math.isfinite(depth) or offset is None:
            return None
        try:
            hfov = self._camera.getFov()
        except Exception:
            hfov = 1.0
        b = self.theta - offset * hfov / 2.0
        return (self.x + depth * math.cos(b), self.y + depth * math.sin(b))

    def _known_victim(self, depth, offset):
        """exp11: is the victim in view one already reported (by either robot)?
        Robot-position radii large enough to stop repeats (1.0-1.5 m) also
        block the neighbour of a found victim: small's v1 and v2 lie 1.4 m
        apart, and exp08 lost v2 in 2 of 3 runs to the 1.5 m homing block
        while homing past v1. Compare body to body instead."""
        if not BODY_ID:
            return self._already_reported_nearby(self.x, self.y, radius=1.0)
        p = self._body_point(depth, offset)
        if p is None:
            return self._already_reported_nearby(self.x, self.y, radius=1.0)
        return any(math.hypot(p[0] - bx, p[1] - by) < BODY_SAME_R_M
                   for bx, by in self.__dict__.setdefault("_body_points", []))

    def _already_reported_nearby(self, x, y, radius=0.5):''')

# B. own report records the body point and broadcasts it
rep("""        self._sup_emitter.send(json.dumps(msg).encode())
        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)""",
"""        self._sup_emitter.send(json.dumps(msg).encode())
        self._reported_positions.append((self.x, self.y))
        _v  = self._last_vision or {}
        _bp = self._body_point(_v.get("depth", float("inf")), _v.get("offset"))
        if _bp is None:
            _bp = (self.x + 0.6 * math.cos(self.theta),
                   self.y + 0.6 * math.sin(self.theta))
        self.__dict__.setdefault("_body_points", []).append(_bp)
        self._last_body_point = _bp
        self._broadcast_victim(self.x, self.y)""")

rep("""        self._squad_tx.send(json.dumps({"type": "victim", "x": x, "y": y}).encode())""",
"""        m = {"type": "victim", "x": x, "y": y}
        bp = getattr(self, "_last_body_point", None)
        if bp is not None:
            m["bx"], m["by"] = bp[0], bp[1]
        self._squad_tx.send(json.dumps(m).encode())""")

# C. partner's body point
rep("""                    vx, vy = float(msg["x"]), float(msg["y"])""",
"""                    vx, vy = float(msg["x"]), float(msg["y"])
                    if "bx" in msg:
                        self.__dict__.setdefault("_body_points", []).append(
                            (float(msg["bx"]), float(msg["by"])))""")

# D. homing gate
rep("""        if self._already_reported_nearby(self.x, self.y,
                                         radius=HOME_BLOCK_R_M):""",
"""        if self._already_reported_nearby(self.x, self.y,
                                         radius=(ROBOT_SAME_R_M if BODY_ID
                                                 else HOME_BLOCK_R_M)):""")
rep("""        conf, depth, _ = self._camera_victim_confidence()
        # Require a FINITE denim depth in range""",
"""        conf, depth, _off = self._camera_victim_confidence()
        if BODY_ID and conf >= APPROACH_ENTER_CONF and self._known_victim(depth, _off):
            self._home_block = "known-body"
            return
        # Require a FINITE denim depth in range""")

# E. direct report gate
rep("""            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
            # Retire the waypoint ONLY""",
"""            if not self._known_victim(depth, offset):
                self.report_victim()
            # Retire the waypoint ONLY""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
