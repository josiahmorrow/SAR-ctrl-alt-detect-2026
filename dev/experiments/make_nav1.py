"""nav1 (approved rewrite, stage 1): log-odds occupancy grid + planner on
hard, chassis-inflated obstacles. usage: python make_nav1.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

# ── constants ────────────────────────────────────────────────────────────────
rep("HARD_WALL_FIRST     = False   # exp01 (25/11) re-measured: worse, reverted",
    "HARD_WALL_FIRST     = True    # nav1: hard, inflated obstacles on a log-odds map\n"
    "                              # (exp01 failed on the binary map: noise sealed doors)")
rep("PARTNER_MASK_M    = 0.7",
    "PARTNER_MASK_M    = 0.7\n"
    "# nav1 (approved 26/11): log-odds occupancy. Every sampled beam lowers the\n"
    "# odds of the cells it passes through and raises the odds of its endpoint;\n"
    "# a cell is a wall while its odds exceed LO_OCC. Anything that moves out of\n"
    "# the way (the partner, a door seen from a tilted chassis) is cleared by the\n"
    "# next few passes instead of staying a wall for the rest of the mission.\n"
    "NAV_LOGODDS       = True\n"
    "LO_HIT            = 0.9\n"
    "LO_PASS           = -0.4\n"
    "LO_MIN, LO_MAX    = -4.0, 4.0\n"
    "LO_OCC            = 1.0\n"
    "LIDAR_PASS_MAX_M  = 5.0    # free-space evidence along each ray up to this\n"
    "NAV_ROBOT_R_M     = 0.18   # chassis radius: obstacles inflated by this are hard\n"
    "NAV_START_FREE_C  = 3      # cells round the start that are never blocked")

# ── grid allocation ─────────────────────────────────────────────────────────
rep("""            self._nav_seen  = np.zeros((N, N), np.uint8)
            self._nav_N     = N""",
"""            self._nav_seen  = np.zeros((N, N), np.uint8)
            self._nav_lo    = np.zeros((N, N), np.float32)   # nav1 log-odds
            self._nav_N     = N""")

# ── hard blocked grid alongside the cost grid ───────────────────────────────
rep("""            near   = cv2.dilate(wall, k)
            cost   = np.ones(wall.shape, dtype=np.float32)
            cost[near > 0] = NAV_NEAR_COST
            cost[wall > 0] = NAV_WALL_COST""",
"""            near   = cv2.dilate(wall, k)
            cost   = np.ones(wall.shape, dtype=np.float32)
            cost[near > 0] = NAV_NEAR_COST
            cost[wall > 0] = NAV_WALL_COST
            # nav1: cells the chassis centre cannot occupy without touching
            # an obstacle (walls and the persistent low-obstacle layer).
            rr = max(1, int(round(NAV_ROBOT_R_M / NAV_RES)))
            kb = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (2 * rr + 1, 2 * rr + 1))
            self._nav_block = cv2.dilate(wall, kb) > 0""")

# ── A*: hard mode uses the inflated grid, keeps the start free ──────────────
rep("""        blocked = (self._nav_wall > 0) if hard else None""",
"""        blocked = None
        if hard:
            blocked = getattr(self, "_nav_block", None)
            if blocked is None or blocked.shape != self._nav_wall.shape:
                blocked = self._nav_wall > 0""")
rep("""                    if blocked is not None and blocked[nr, nc] and (nr, nc) != g:
                        continue""",
"""                    if (blocked is not None and blocked[nr, nc] and (nr, nc) != g
                            and max(abs(nr - s[0]), abs(nc - s[1]))
                            > NAV_START_FREE_C):
                        continue""")

# ── plan: move a blocked goal to the nearest free cell ──────────────────────
rep("""        s, g = self._w2c(self.x, self.y), self._w2c(gx, gy)
        # Prefer a route that does not cross known walls""",
"""        s, g = self._w2c(self.x, self.y), self._w2c(gx, gy)
        if HARD_WALL_FIRST and NAV_LOGODDS:
            g = self._free_goal(g)
        # Prefer a route that does not cross known walls""")

rep("""    def _route_cost(self, p, q):""",
'''    def _free_goal(self, g, radius_c=10):
        """nav1: estimates often sit in a wall or a sofa; aim the hard search
        at the nearest cell the chassis can actually occupy (within 1 m)."""
        self._cost_grid()
        blk = getattr(self, "_nav_block", None)
        g = self._clamp_rc(g)
        if blk is None or not blk[g]:
            return g
        n = self._nav_N
        r0, r1 = max(0, g[0] - radius_c), min(n, g[0] + radius_c + 1)
        c0, c1 = max(0, g[1] - radius_c), min(n, g[1] + radius_c + 1)
        free = np.argwhere(~blk[r0:r1, c0:c1])
        if free.size == 0:
            return g
        d = (free[:, 0] + r0 - g[0]) ** 2 + (free[:, 1] + c0 - g[1]) ** 2
        k = int(np.argmin(d))
        return (int(free[k, 0] + r0), int(free[k, 1] + c0))

    def _route_cost(self, p, q):''')

# ── lidar update: log-odds ──────────────────────────────────────────────────
rep("""        step     = max(1, n_beams // LIDAR_MAP_BEAMS)
        for i in range(0, n_beams, step):
            r   = ranges[i]
            hit = r if r < 1e5 else float('inf')""",
"""        step     = max(1, n_beams // LIDAR_MAP_BEAMS)
        if NAV_LOGODDS:
            self._lidar_logodds(ranges, n_beams, step, pxy, now)
            return
        for i in range(0, n_beams, step):
            r   = ranges[i]
            hit = r if r < 1e5 else float('inf')""")

rep("""    # ── End-of-mission learned-map deliverable""",
'''    def _lidar_logodds(self, ranges, n_beams, step, pxy, now):
        """nav1: one log-odds update per scan. Each sampled beam contributes
        one pass observation to every cell it crosses (from 0.25 m to 0.12 m
        short of its return, up to LIDAR_PASS_MAX_M) and one hit observation
        to its endpoint (returns nearer than LIDAR_PAINT_MAX_M, partner
        masked). A cell counts once per scan however many beams cross it."""
        N, lo = self._nav_N, self._nav_lo
        passes, hits = [], []
        for i in range(0, n_beams, step):
            r   = ranges[i]
            hit = r if (r == r and r < 1e5) else float('inf')
            ang = self.theta + (math.pi - 2.0 * math.pi * i / n_beams)
            ca, sa = math.cos(ang), math.sin(ang)
            limit = min(hit - 0.12, LIDAR_PASS_MAX_M)
            d = 0.25
            while d < limit:
                rr, cc = self._w2c(self.x + d * ca, self.y + d * sa)
                if 0 <= rr < N and 0 <= cc < N:
                    passes.append(rr * N + cc)
                d += NAV_RES * 0.7
            if hit < LIDAR_PAINT_MAX_M:
                hx, hy = self.x + hit * ca, self.y + hit * sa
                if pxy is not None and math.hypot(hx - pxy[0],
                                                  hy - pxy[1]) < PARTNER_MASK_M:
                    continue
                rr, cc = self._w2c(hx, hy)
                if 0 <= rr < N and 0 <= cc < N:
                    hits.append(rr * N + cc)
        if not passes and not hits:
            return
        p = np.unique(np.asarray(passes, dtype=np.int64)) if passes else np.zeros(0, np.int64)
        h = np.unique(np.asarray(hits, dtype=np.int64)) if hits else np.zeros(0, np.int64)
        if h.size:
            p = np.setdiff1d(p, h, assume_unique=True)
        flat_lo = lo.reshape(-1)
        flat_lo[p] = np.maximum(LO_MIN, flat_lo[p] + LO_PASS)
        flat_lo[h] = np.minimum(LO_MAX, flat_lo[h] + LO_HIT)
        touched = np.concatenate([p, h])
        self._nav_seen.reshape(-1)[touched] = 1
        wall = self._nav_wall.reshape(-1)
        new = (flat_lo[touched] > LO_OCC).astype(np.uint8)
        diff = new != wall[touched]
        if diff.any():
            painted = bool((new[diff] > wall[touched][diff]).any())
            wall[touched[diff]] = new[diff]
            self._cost_dirty = True
            if painted and now - self._last_plan_time > 1.5:
                self._path = None                 # re-plan around new walls

    # ── End-of-mission learned-map deliverable''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
