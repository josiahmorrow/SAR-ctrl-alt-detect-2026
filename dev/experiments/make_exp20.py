"""exp20: every body the camera sees is a low obstacle, not only reported ones.
usage: make_exp20.py <in> <out>

exp07 paints a victim's footprint into the persistent low layer when it is
REPORTED. A body that is seen but not homed on (blocked: near a prior
report, partner zone, cooldown; or simply not yet reached) stays invisible
to the planner, because the lidar plane passes over it. exp16 small s5
robot2: after reporting v1 it sees v2 at 1.0 m (conf 0.95, blocked
near-prior-report), the planned leg to the next waypoint runs straight over
v2, the robot climbs it (tilt 19-26 deg) and flips at t=104.
Every 0.5 s while not homing: if a candidate passes the vision gate with a
finite depth <= SEEN_BODY_MAX_M and the chassis is level, paint its
footprint (camera bearing, measured depth, width-dependent extent) into the
persistent low layer and force a replan.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "SEEN_BODY           = True   # exp20: paint every seen body as low obstacle\n"
    "SEEN_BODY_MAX_M     = 2.0    # exp20: only well-measured depths\n"
    "SEEN_BODY_EVERY_S   = 0.5    # exp20\n"
    "SECOND_LOOK         = True")

rep('                             "width_m": width_m}\n        return base_conf, min_depth, x_offset',
    '                             "width_m": width_m, "x_offset": x_offset}\n'
    '        return base_conf, min_depth, x_offset')

rep("""    def _ir_front_m(self):""",
    """    def _seen_body_step(self):
        \"\"\"exp20: paint the footprint of any body in view (see header).\"\"\"
        now = self.robot.getTime()
        if now < getattr(self, "_seen_next", 0.0):
            return
        self._seen_next = now + SEEN_BODY_EVERY_S
        if (self._homing or self._repositioning is not None
                or self._nav_wall is None or self._tilt_deg_now > 4.0):
            return
        conf, depth, xo = self._camera_victim_confidence()
        if conf < APPROACH_ENTER_CONF or not (math.isfinite(depth)
                                              and depth <= SEEN_BODY_MAX_M):
            return
        try:
            hf = float(self._camera.getFov())
        except Exception:
            hf = 1.0
        bearing = -math.atan(xo * math.tan(hf * 0.5))
        wm = (self._last_vision or {}).get("width_m", float("nan"))
        wm = wm if math.isfinite(wm) else 0.6
        half = min(0.9, max(0.3, wm / 2.0 + 0.1))
        deep = 0.6 if wm > 0.9 else 1.3
        th = self.theta + bearing
        ca, sa = math.cos(th), math.sin(th)
        a = max(0.30, depth - 0.05)
        painted = 0
        while a <= depth + deep:
            l = -half
            while l <= half:
                r, c = self._w2c(self.x + a * ca - l * sa, self.y + a * sa + l * ca)
                if not (0 <= r < self._nav_N and 0 <= c < self._nav_N):
                    l += NAV_RES
                    continue
                low = getattr(self, "_nav_low", None)
                if low is None or not low[r, c]:
                    painted += 1
                self._low_mark(r, r + 1, c, c + 1)
                l += NAV_RES
            a += NAV_RES
        if painted > 4:
            self._path = None

    def _ir_front_m(self):""")

rep("""            self._vision_telemetry()
""", """            self._vision_telemetry()
            if SEEN_BODY:
                self._seen_body_step()
""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
