"""nav4 (planner replacement, approved by Joe 27/11 04:03), stage on top of
nav1 (log-odds map + hard chassis-inflated planner). usage: make_nav4.py <in> <out>

The old planner treated space the lidar has not seen as open floor at cost 1
and every known wall as a finite cost, so routes ran straight through walls
not yet seen and through walls already seen when the detour was long; the
robot found out by bumping, fell back to Bug2 wall-following, and timed out
(exp23: ~16 waypoint timeouts per run). nav4:
  * three-state map: unknown cells cost NAV_UNKNOWN_COST (> free), so a
    route prefers floor the lidar has seen and only crosses unknown space
    where it must (the target is usually in unknown space);
  * walls and the persistent low layer (bodies, IR, tilt) are hard and
    inflated by the chassis radius (nav1); goals inside them move to the
    nearest free cell (nav1);
  * the route is re-checked every PATH_CHECK_S against the current map and
    re-planned the moment it crosses a blocked cell, instead of waiting for
    the 5 s replan or a bump;
  * Bug2 wall-following is off: with a hard map it only fought the planner;
  * bounded-suboptimal A* (heuristic weight NAV_H_WEIGHT) keeps the search
    inside its budget now that costs are no longer all 1.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("WALL_FOLLOW          = True", "WALL_FOLLOW          = False  # nav4: off (hard map + replanning)")
rep("SECOND_LOOK         = True",
    "NAV4                = True   # nav4 planner (see make_nav4.py)\n"
    "NAV_UNKNOWN_COST    = 1.5    # nav4: per-cell cost of unseen space\n"
    "NAV_H_WEIGHT        = 1.3    # nav4: bounded-suboptimal A*\n"
    "PATH_CHECK_S        = 0.5    # nav4: route re-validation period\n"
    "PATH_CHECK_AHEAD_M  = 2.5    # nav4: how far along the route to check\n"
    "SECOND_LOOK         = True")

# unknown space costs more than seen floor
rep("""            cost   = np.ones(wall.shape, dtype=np.float32)
            cost[near > 0] = NAV_NEAR_COST
            cost[wall > 0] = NAV_WALL_COST
            # nav1""",
    """            cost   = np.ones(wall.shape, dtype=np.float32)
            if NAV4 and getattr(self, "_nav_seen", None) is not None:
                cost[self._nav_seen == 0] = NAV_UNKNOWN_COST
            cost[near > 0] = NAV_NEAR_COST
            cost[wall > 0] = NAV_WALL_COST
            # nav1""")

# newly seen cells change the cost grid even when no wall changed
rep("""        touched = np.concatenate([p, h])
        self._nav_seen.reshape(-1)[touched] = 1""",
    """        touched = np.concatenate([p, h])
        _sv = self._nav_seen.reshape(-1)
        if NAV4 and not _sv[touched].all():
            self._cost_dirty = True
        _sv[touched] = 1""")

# weighted heuristic
rep("""                        h = math.hypot(nr - g[0], nc - g[1])   # admissible: min cost 1""",
    """                        h = math.hypot(nr - g[0], nc - g[1])   # admissible: min cost 1
                        if NAV4:
                            h *= NAV_H_WEIGHT""")

# route re-validation helper
rep("""    def _route_cost(self, p, q):""",
    '''    def _path_blocked(self):
        """nav4: True if the next PATH_CHECK_AHEAD_M of the current route
        crosses a cell the chassis cannot occupy on the current map. The
        first 0.3 m is skipped (the robot may legitimately start beside a
        wall)."""
        blk = getattr(self, "_nav_block", None)
        if blk is None or not self._path:
            return False
        self._cost_grid()
        blk = self._nav_block
        n = self._nav_N
        px, py = self.x, self.y
        travelled = 0.0
        for i in range(self._path_idx, len(self._path)):
            qx, qy = self._path[i]
            seg = math.hypot(qx - px, qy - py)
            k = max(1, int(seg / NAV_RES))
            for j in range(1, k + 1):
                t = j / k
                x, y = px + (qx - px) * t, py + (qy - py) * t
                dd = travelled + seg * t
                if dd < 0.3:
                    continue
                r, c = self._w2c(x, y)
                if 0 <= r < n and 0 <= c < n and blk[r, c]:
                    return True
            travelled += seg
            if travelled > PATH_CHECK_AHEAD_M:
                break
            px, py = qx, qy
        return False

    def _route_cost(self, p, q):''')

# re-validate before following the route
rep("""            # ── Plan a route to the goal, follow it as sub-waypoints ──""",
    """            if (NAV4 and self._path and self._planned_goal == (tx, ty)
                    and now >= getattr(self, "_pchk_next", 0.0)):
                self._pchk_next = now + PATH_CHECK_S
                if self._path_blocked():
                    self._path = None
            # ── Plan a route to the goal, follow it as sub-waypoints ──""")

# only re-validate routes that came from the hard search
rep("""        cells = None
        if HARD_WALL_FIRST:
            cells, _ = self._astar(s, g, hard=True)
        if not cells:
            cells, _ = self._astar(s, g)""",
    """        cells = None
        self._path_hard = False
        if HARD_WALL_FIRST:
            cells, _ = self._astar(s, g, hard=True)
            self._path_hard = bool(cells)
        if not cells:
            cells, _ = self._astar(s, g)""")
rep("""            if (NAV4 and self._path and self._planned_goal == (tx, ty)""",
    """            if (NAV4 and self._path and self._planned_goal == (tx, ty)
                    and getattr(self, "_path_hard", False)""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
