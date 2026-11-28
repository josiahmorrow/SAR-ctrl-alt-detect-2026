"""co2: share the reported BODY position, not only the reporter's pose.
usage: make_co2.py <in> <out>

Observed at 180 s on large: the second robot re-reports a victim the first
has already reported ('dup' false reports of v4, 0.46-0.99 m from it). The
'victim' message carries the reporter's pose; the reporter stands about
0.75 m off the body on one side, so a partner arriving from the far side
can be more than 1.0 m from that pose (outside the report dedup) while
within 1.0 m of the same victim.

co2 adds optional fields bx, by to the 'victim' message: the reported
body's ground position (pose + (depth + 0.3) along the camera bearing, the
same projection REPO_MATCH uses). The receiver keeps these and
  * does not start homing on a candidate whose own projected position is
    within BODY_SAME_M of a partner-reported body ("partner-body");
  * refuses, inside report_victim, a report on such a candidate.
The test is on the CANDIDATE's position, not the robot's, so it does not
widen the no-report zone around the partner's pose: an adjacent victim more
than BODY_SAME_M from the reported body stays reportable.
A message without bx/by (unpatched partner, or a report made with no
candidate in view) behaves exactly as before.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("HOME_BLOCK_R_M      = 1.5\n",
    "HOME_BLOCK_R_M      = 1.5\n"
    "CO2                 = True   # co2: partner shares the reported body position\n"
    "BODY_SAME_M         = 0.8    # co2: two projections of one body\n")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._co2_bodies       = []      # co2: bodies the partner reported\n")

rep("    def _broadcast_victim(self, x, y):\n", '''    def _co2_body_point(self):
        """co2: ground position of the candidate in view, or None."""
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

    def _co2_partner_body_hit(self):
        """co2: the candidate in view is a body the partner already reported."""
        if not CO2 or not self._co2_bodies:
            return False
        bp = self._co2_body_point()
        if bp is None:
            return False
        return any(math.hypot(bp[0] - bx, bp[1] - by) < BODY_SAME_M
                   for bx, by in self._co2_bodies)

    def _broadcast_victim(self, x, y):
''')

# send
rep('        self._squad_tx.send(json.dumps({"type": "victim", "x": x, "y": y}).encode())\n',
    '        _m = {"type": "victim", "x": x, "y": y}\n'
    '        if CO2:\n'
    '            _b = self._co2_body_point()\n'
    '            if _b is not None:\n'
    '                _m["bx"], _m["by"] = round(_b[0], 2), round(_b[1], 2)\n'
    '        self._squad_tx.send(json.dumps(_m).encode())\n')

# receive
rep('                    vx, vy = float(msg["x"]), float(msg["y"])\n',
    '                    vx, vy = float(msg["x"]), float(msg["y"])\n'
    '                    if CO2 and "bx" in msg and "by" in msg:\n'
    '                        try:\n'
    '                            _b = (float(msg["bx"]), float(msg["by"]))\n'
    '                            if not any(math.hypot(_b[0] - ox, _b[1] - oy) < 0.2\n'
    '                                       for ox, oy in self._co2_bodies):\n'
    '                                self._co2_bodies.append(_b)\n'
    '                        except (TypeError, ValueError):\n'
    '                            pass\n')

# no homing on a partner-reported body
rep('''        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''', '''        if self._co2_partner_body_hit():
            self._home_block = "partner-body"
            return
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''')

# no report on a partner-reported body (covers every caller)
rep('''        if not self._sup_emitter:
            return
        try:
''', '''        if not self._sup_emitter:
            return
        if self._co2_partner_body_hit():
            print(f"[{self.robot_id}] co2 report withheld - the partner already "
                  f"reported this body")
            return
        try:
''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
