"""exp29: do not re-home on a body this robot has already reported.
usage: make_exp29.py <in> <out>

b180 (180 s baseline) large L0: robot1 reports v4 at 43 s, then spends the
remaining 137 s within ~1 m of it: each time it gets more than
HOME_BLOCK_R_M (1.5 m) from its report pose it sees the same body, homes
on it again, is refused at report time by the 1.0 m dedup, and repeats.
Its second flyover waypoint is never reached. Same loop for robot2 after
its report. HOME_BLOCK_R_M tests the ROBOT's position; a body 1.7 m long
is visible from well outside that.

exp29 records, at each own report from the homing branch, the reported
body's position (pose + (depth + 0.3) along the heading, as REPO_MATCH),
and refuses to start homing when the candidate in view projects (same
formula along its camera bearing) within BODY_BLOCK_M of a recorded body.
It only ever blocks homing, so it cannot create a duplicate report.
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
    "BODY_BLOCK          = True   # exp29: no re-homing on a reported body\n"
    "BODY_BLOCK_M        = 1.2    # exp29\n")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._reported_bodies  = []      # exp29\n")

# record the body point when the homing branch reports
rep('''            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
            # Retire the waypoint ONLY when the find is at the waypoint we were''',
    '''            _n29 = len(self._reported_positions)
            if not self._already_reported_nearby(self.x, self.y, radius=1.0):
                self.report_victim()
            if BODY_BLOCK and len(self._reported_positions) > _n29:
                self._reported_bodies.append(
                    (self.x + (depth + 0.3) * math.cos(self.theta),
                     self.y + (depth + 0.3) * math.sin(self.theta)))
            # Retire the waypoint ONLY when the find is at the waypoint we were''')

# block homing on a recorded body
rep('''        conf, depth, _ = self._camera_victim_confidence()
        # Require a FINITE denim depth in range: a denim patch with no depth''',
    '''        conf, depth, _xo29 = self._camera_victim_confidence()
        if BODY_BLOCK and self._reported_bodies and math.isfinite(depth):
            try:
                _hf = float(self._camera.getFov())
            except Exception:
                _hf = 1.0
            _th = self.theta - math.atan(_xo29 * math.tan(_hf * 0.5))
            _bx = self.x + (depth + 0.3) * math.cos(_th)
            _by = self.y + (depth + 0.3) * math.sin(_th)
            if any(math.hypot(_bx - px, _by - py) < BODY_BLOCK_M
                   for px, py in self._reported_bodies):
                self._home_block = "reported-body"
                return
        # Require a FINITE denim depth in range: a denim patch with no depth''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
