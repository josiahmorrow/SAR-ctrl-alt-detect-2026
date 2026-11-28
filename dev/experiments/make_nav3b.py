"""nav3b: depth-camera low obstacles with free-space clearing (log-odds), so
things that move (the partner, a door) do not stay obstacles.
usage: make_nav3b.py <ps_nav3_on08.py> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("DEPTH_LOW_EVERY_S   = 0.5",
    "DEPTH_LOW_EVERY_S   = 0.5\n"
    "DLOW_HIT, DLOW_FREE, DLOW_OCC = 1.0, -0.6, 1.5   # nav3b log-odds\n"
    "DLOW_FLOOR_H_M      = 0.03   # returns this close to z=0 are floor (clears)")

# floor evidence + log-odds instead of a permanent mark
rep("""        m = ok & (hgt > DEPTH_LOW_H_M[0]) & (hgt < DEPTH_LOW_H_M[1])
        if int(m.sum()) < DEPTH_LOW_MIN_PX:
            return""",
"""        m = ok & (hgt > DEPTH_LOW_H_M[0]) & (hgt < DEPTH_LOW_H_M[1])
        fl = ok & (np.abs(hgt) < DLOW_FLOOR_H_M)""")

rep("""        ct, st = math.cos(self.theta), math.sin(self.theta)
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
                self._path = None""",
"""        ct, st = math.cos(self.theta), math.sin(self.theta)
        n = self._nav_N

        def cells_of(mask, min_px):
            wx = self.x + fwd[mask] * ct - lat[mask] * st
            wy = self.y + fwd[mask] * st + lat[mask] * ct
            rr = np.round(n / 2 - wy / NAV_RES).astype(np.int64)
            cc = np.round(n / 2 + wx / NAV_RES).astype(np.int64)
            inb = (rr >= 0) & (rr < n) & (cc >= 0) & (cc < n)
            if not inb.any():
                return np.zeros(0, np.int64)
            cells, counts = np.unique(rr[inb] * n + cc[inb], return_counts=True)
            return cells[counts >= min_px]

        hit  = cells_of(m, DEPTH_LOW_MIN_PX)
        free = cells_of(fl, DEPTH_LOW_MIN_PX)
        if hit.size:
            free = np.setdiff1d(free, hit, assume_unique=True)
        if getattr(self, "_dlow_lo", None) is None:
            self._dlow_lo = np.zeros(self._nav_wall.shape, np.float32)
            self._nav_dlow = np.zeros_like(self._nav_wall)
        lo = self._dlow_lo.reshape(-1)
        lo[hit]  = np.minimum(4.0, lo[hit] + DLOW_HIT)
        lo[free] = np.maximum(-4.0, lo[free] + DLOW_FREE)
        touched = np.concatenate([hit, free])
        if touched.size == 0:
            return
        new = (lo[touched] > DLOW_OCC).astype(np.uint8)
        dl = self._nav_dlow.reshape(-1)
        diff = new != dl[touched]
        if diff.any():
            dl[touched[diff]] = new[diff]
            self._cost_dirty = True
            if now - self._last_plan_time > 1.5:
                self._path = None""")

# include the depth layer in the cost grid (separately from the permanent layer)
rep("""            if LOW_PERSIST and low is not None and low.shape == wall.shape:
                wall = np.maximum(wall, low)""",
"""            if LOW_PERSIST and low is not None and low.shape == wall.shape:
                wall = np.maximum(wall, low)
            dlow = getattr(self, "_nav_dlow", None)
            if dlow is not None and dlow.shape == wall.shape:
                wall = np.maximum(wall, dlow)""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
