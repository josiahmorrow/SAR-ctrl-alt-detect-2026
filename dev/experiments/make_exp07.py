"""exp07: low obstacles persist; a confirmed victim's body is one.
usage: python make_exp07.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:60])
    src = src.replace(old, new)

# 1. constants
rep("RECOVER_PULSE_S    = 0.6",
    "LOW_PERSIST        = True   # exp07: IR/tilt/victim obstacles survive lidar erase\n"
    "RECOVER_PULSE_S    = 0.6")

# 2. cost grid includes the persistent low layer
rep("""            near   = cv2.dilate(self._nav_wall, k)
            cost   = np.ones(self._nav_wall.shape, dtype=np.float32)
            cost[near > 0]           = NAV_NEAR_COST
            cost[self._nav_wall > 0] = NAV_WALL_COST""",
"""            wall   = self._nav_wall
            low    = getattr(self, "_nav_low", None)
            if LOW_PERSIST and low is not None and low.shape == wall.shape:
                wall = np.maximum(wall, low)
            near   = cv2.dilate(wall, k)
            cost   = np.ones(wall.shape, dtype=np.float32)
            cost[near > 0] = NAV_NEAR_COST
            cost[wall > 0] = NAV_WALL_COST""")

# 3. IR/tilt low obstacles also go to the persistent layer
rep("""        if 1 <= r < n - 1 and 1 <= c < n - 1:
            self._nav_wall[r - 1:r + 2, c - 1:c + 2] = 1
            self._nav_seen[r - 1:r + 2, c - 1:c + 2] = 1""",
"""        if 1 <= r < n - 1 and 1 <= c < n - 1:
            self._nav_wall[r - 1:r + 2, c - 1:c + 2] = 1
            self._nav_seen[r - 1:r + 2, c - 1:c + 2] = 1
            self._low_mark(r - 1, r + 2, c - 1, c + 2)""")

# 4. helper + victim footprint, placed before _ir_front_m
rep("""    def _ir_front_m(self):""",
'''    def _low_mark(self, r0, r1, c0, c1):
        """exp07: persistent low-obstacle layer. The lidar plane passes over
        bodies, kerbs and rubble, so its free-space erase must not undo what
        the IR, the tilt guard or a confirmed victim told us is there."""
        if not LOW_PERSIST or self._nav_wall is None:
            return
        if getattr(self, "_nav_low", None) is None:
            self._nav_low = np.zeros_like(self._nav_wall)
        self._nav_low[max(0, r0):r1, max(0, c0):c1] = 1
        self._cost_dirty = True

    def _paint_victim_body(self):
        """exp07: a confirmed victim is a low obstacle the robot must route
        around, not over (small exp05b/exp06: robots beached on the body they
        had just reported, on the way to the next waypoint). Footprint ahead
        of the camera: from the measured denim depth, extent from the blob's
        measured width (broadside = wide and shallow, end-on = narrow, deep)."""
        if not LOW_PERSIST or self._nav_wall is None:
            return
        v  = self._last_vision or {}
        d0 = v.get("depth", float("inf"))
        d0 = d0 if math.isfinite(d0) and d0 < 1.5 else 0.5
        wm = v.get("width_m", float("nan"))
        wm = wm if math.isfinite(wm) else 0.6
        half = min(0.9, max(0.3, wm / 2.0 + 0.1))
        deep = 0.6 if wm > 0.9 else 1.3
        ca, sa = math.cos(self.theta), math.sin(self.theta)
        a = max(0.25, d0 - 0.1)
        while a <= d0 + deep:
            l = -half
            while l <= half:
                r, c = self._w2c(self.x + a * ca - l * sa, self.y + a * sa + l * ca)
                self._low_mark(r, r + 1, c, c + 1)
                l += NAV_RES
            a += NAV_RES
        self._path = None

    def _ir_front_m(self):''')

# 5. paint on report
rep("""        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)""",
"""        self._reported_positions.append((self.x, self.y))
        self._broadcast_victim(self.x, self.y)
        self._paint_victim_body()""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
