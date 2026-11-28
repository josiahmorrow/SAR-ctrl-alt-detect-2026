"""nav3: map low obstacles (bodies, rubble, kerbs) from the depth camera into
the persistent low-obstacle layer. usage: make_nav3.py <in> <out>
Needs the exp07 low layer (_low_mark) in <in>."""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("SECOND_LOOK         = True",
    "DEPTH_LOW_MAP       = True   # nav3: depth-camera low obstacles into the map\n"
    "DEPTH_LOW_H_M       = (0.07, 0.30)  # height band below the lidar plane\n"
    "DEPTH_LOW_RANGE_M   = 1.8    # trust depth returns nearer than this\n"
    "DEPTH_LOW_MIN_PX    = 4      # pixels per cell before it counts\n"
    "DEPTH_LOW_EVERY_S   = 0.5\n"
    "SECOND_LOOK         = True")

rep("""            self._update_map_from_lidar()
            self._learned_map_step()""",
"""            self._update_map_from_lidar()
            self._depth_low_step()
            self._learned_map_step()""")

rep("""    def _read_tilt(self):""",
'''    def _depth_low_step(self):
        """nav3: the lidar plane passes over lying bodies, kerbs and rubble;
        the IR sees them only dead ahead at 0.35 m and the tilt guard only
        after the climb has begun. The depth camera sees them 1-2 m out.
        Project its pixels to the floor frame and mark every map cell with
        enough returns 7-30 cm above the floor as a low obstacle, so the
        planner routes round bodies and debris before touching them.
        Level chassis only (a tilted camera puts the floor in that band)."""
        if not DEPTH_LOW_MAP or not self._depth_camera or self._nav_wall is None:
            return
        now = self.robot.getTime()
        if now < getattr(self, "_dlow_next", 0.0) or self._tilt_deg_now > 4.0:
            return
        self._dlow_next = now + DEPTH_LOW_EVERY_S
        try:
            dw = self._depth_camera.getWidth()
            dh = self._depth_camera.getHeight()
            d = np.array(self._depth_camera.getRangeImage(),
                         dtype=np.float32).reshape((dh, dw))
            hf = float(self._depth_camera.getFov())
        except Exception:
            return
        vf = 2.0 * math.atan(math.tan(hf * 0.5) * dh / float(dw))
        fx = (dw * 0.5) / math.tan(hf * 0.5)
        fy = (dh * 0.5) / math.tan(vf * 0.5)
        cols = np.arange(dw, dtype=np.float32)[None, :]
        rows = np.arange(dh, dtype=np.float32)[:, None]
        az   = np.arctan2(dw * 0.5 - cols, fx)          # left positive
        el   = np.arctan2(dh * 0.5 - rows, fy)
        ok   = np.isfinite(d) & (d > 0.15) & (d < DEPTH_LOW_RANGE_M)
        if not ok.any():
            return
        # Planar depth (same model as the victim gate's height test)
        fwd  = d
        lat  = d * np.tan(az)
        hgt  = CAM_HEIGHT_M + d * np.tan(el)
        m = ok & (hgt > DEPTH_LOW_H_M[0]) & (hgt < DEPTH_LOW_H_M[1])
        if int(m.sum()) < DEPTH_LOW_MIN_PX:
            return
        ct, st = math.cos(self.theta), math.sin(self.theta)
        wx = self.x + fwd[m] * ct - lat[m] * st
        wy = self.y + fwd[m] * st + lat[m] * ct
        if self._partner_x is not None:              # the partner moves on
            keep = np.hypot(wx - self._partner_x, wy - self._partner_y) > 0.6
            wx, wy = wx[keep], wy[keep]
            if wx.size == 0:
                return
        n = self._nav_N
        rr = np.round(n / 2 - wy / NAV_RES).astype(np.int64)
        cc = np.round(n / 2 + wx / NAV_RES).astype(np.int64)
        inb = (rr >= 0) & (rr < n) & (cc >= 0) & (cc < n)
        if not inb.any():
            return
        flat = rr[inb] * n + cc[inb]
        cells, counts = np.unique(flat, return_counts=True)
        hit = cells[counts >= DEPTH_LOW_MIN_PX]
        if hit.size == 0:
            return
        if getattr(self, "_nav_low", None) is None:
            self._nav_low = np.zeros_like(self._nav_wall)
        before = int(self._nav_low.sum())
        self._nav_low.reshape(-1)[hit] = 1
        if int(self._nav_low.sum()) != before:
            self._cost_dirty = True
            if now - self._last_plan_time > 1.5:
                self._path = None

    def _read_tilt(self):''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
