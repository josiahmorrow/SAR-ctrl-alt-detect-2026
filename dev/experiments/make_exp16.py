"""exp16: after a report, back out along the robot's own inbound trail.
usage: make_exp16.py <in> <out>

Physical argument: a robot parked against a lying body (often between two, or
a body and furniture) is surrounded by obstacles the lidar plane passes over.
The only floor KNOWN to be traversable at chassis height is the track the
robot just drove in on. exp07's 1 s straight reverse covers ~0.2 m and only
along the current heading; the next leg's turn-and-go then presses against
the body until the chassis mounts it (exp08 small s1 robot2: reported at
t=34.8 between v1 and v2, IR 0.17-0.45 m for 6 s, mounted at t=43, flipped).
Retrace BACKOUT_M of trail in reverse, steering the rear onto it.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "BACKOUT             = True   # exp16: retrace inbound trail after a report\n"
    "BACKOUT_M           = 0.7    # exp16: trail length to retrace\n"
    "BACKOUT_TIMEOUT_S   = 6.0    # exp16\n"
    "SECOND_LOOK         = True")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._trail            = []      # exp16: recent level-ground poses\n"
    "        self._backout          = None    # exp16: points to retrace\n"
    "        self._backout_until    = 0.0     # exp16\n")

# report: back out along the trail instead of the straight reverse
rep("""        if LOW_PERSIST:
            self._tilt_escape_until = max(self._tilt_escape_until,
                                          self.robot.getTime() + POST_REPORT_BACK_S)""",
    """        if BACKOUT and self._start_backout():
            pass
        elif LOW_PERSIST:
            self._tilt_escape_until = max(self._tilt_escape_until,
                                          self.robot.getTime() + POST_REPORT_BACK_S)""")

# helpers before report_victim
rep("""    def report_victim(self):""",
    """    def _trail_step(self):
        \"\"\"exp16: record level-ground poses every 5 cm (last ~10 m).\"\"\"
        if self._tilt_deg_now > 6.0:
            return
        if (not self._trail or math.hypot(self.x - self._trail[-1][0],
                                          self.y - self._trail[-1][1]) > 0.05):
            self._trail.append((self.x, self.y))
            if len(self._trail) > 200:
                del self._trail[:-200]

    def _start_backout(self):
        pts, acc = [], 0.0
        px, py = self.x, self.y
        for tx, ty in reversed(self._trail):
            acc += math.hypot(tx - px, ty - py)
            px, py = tx, ty
            if math.hypot(tx - self.x, ty - self.y) > 0.08:
                pts.append((tx, ty))
            if acc >= BACKOUT_M:
                break
        if not pts:
            return False
        # Thin to ~10 cm spacing; the last point is the goal.
        thin = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - thin[-1][0], p[1] - thin[-1][1]) >= 0.10:
                thin.append(p)
        if thin[-1] != pts[-1]:
            thin.append(pts[-1])
        self._backout = thin
        self._backout_until = self.robot.getTime() + BACKOUT_TIMEOUT_S
        print(f"[{self.robot_id}] Backing out {acc:.2f} m along inbound trail")
        return True

    def _backout_step(self):
        \"\"\"Reverse with the rear steered onto the next trail point.
        Returns True while the back-out owns the wheels.\"\"\"
        if not self._backout or self.robot.getTime() > self._backout_until:
            if self._backout is not None:
                self.stop()
            self._backout = None
            return False
        tx, ty = self._backout[0]
        if math.hypot(tx - self.x, ty - self.y) < 0.07:
            self._backout.pop(0)
            if not self._backout:
                self.stop()
                self._backout = None
                return False
            tx, ty = self._backout[0]
        want = math.atan2(ty - self.y, tx - self.x) + math.pi   # rear first
        err = (want - self.theta + math.pi) % (2.0 * math.pi) - math.pi
        turn = max(-0.25, min(0.25, 0.8 * err))
        base = -0.30 if abs(err) < 0.8 else -0.12
        self.set_speeds(MAX_SPEED * (base - turn), MAX_SPEED * (base + turn))
        return True

    def report_victim(self):""")

# trail recording, every tick after the tilt estimate update
rep("""            self._tilt_deg_now += _k * (self._read_tilt() - self._tilt_deg_now)
""", """            self._tilt_deg_now += _k * (self._read_tilt() - self._tilt_deg_now)
            if BACKOUT:
                self._trail_step()
""")

# main loop: back-out runs right after the tilt escape (tilt guard keeps priority)
rep("""            if now_g < self._tilt_escape_until:
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                continue
""", """            if now_g < self._tilt_escape_until:
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                continue
            if BACKOUT and self._backout and self._backout_step():
                continue
""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
