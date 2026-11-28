"""co4: share newly learned wall cells so both planners use both lidars.
usage: make_co4.py <in> <out>

Observed at 180 s on large: robot1 spends 70 s trying to reach a target
behind a wall, on a route robot2 has already mapped. Each robot learns the
layout only from its own lidar.

co4 sends a new channel-73 message every WALL_SHARE_EVERY_S:
    {"type": "walls", "id": "<robot id>", "c": [[x10, y10], ...]}
with up to WALL_SHARE_MAX wall cells this robot's lidar has painted since
the last message (coordinates in decimetres in the shared origin-mat
frame, so the two grids need not be the same size). The newest cells near
the robot go first; the rest follow in later messages.

The receiver marks each cell as wall in its planning grid, except cells
within WALL_SELF_CLEAR_M of itself (the partner may have seen THIS robot)
and cells outside its grid. Its own lidar still erases any cell it sees
through, exactly as for its own phantom walls, so a misplaced cell does
not persist in space it observes. Received cells are never re-broadcast.
`_nav_seen` (coverage / learned-map bookkeeping) is deliberately NOT
touched: only the planner's wall layer is shared.

RISK (flagged): the two robots' odometry frames can disagree after wheel
slip or a tilt episode; a wall shared from a drifted frame is misplaced in
the receiver's map until its own lidar sees through it. Test this one
last and judge it on its own.
An unpatched or silent partner changes nothing.
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
    "CO4                 = True   # co4: share learned wall cells\n"
    "WALL_SHARE_EVERY_S  = 2.0    # co4\n"
    "WALL_SHARE_MAX      = 80     # co4: cells per message (about 1 kB)\n"
    "WALL_SELF_CLEAR_M   = 0.6    # co4: never wall in the receiver's own footprint\n")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._co4_known        = None    # co4: wall cells already shared/received\n"
    "        self._co4_next_t       = 0.0     # co4\n")

rep("    def _broadcast_victim(self, x, y):\n", '''    def _co4_share_walls(self):
        """co4: broadcast wall cells painted since the last message."""
        if not CO4 or not self._squad_tx or self._nav_wall is None:
            return
        now = self.robot.getTime()
        if now < self._co4_next_t:
            return
        self._co4_next_t = now + WALL_SHARE_EVERY_S
        try:
            wall = self._nav_wall.astype(bool)
            if self._co4_known is None or self._co4_known.shape != wall.shape:
                # First call: what is there now is the shared starting grid.
                self._co4_known = wall.copy()
                return
            self._co4_known &= wall                # erased cells may be re-sent
            new = np.argwhere(wall & ~self._co4_known)
            if len(new) == 0:
                return
            if len(new) > WALL_SHARE_MAX:
                mr, mc = self._w2c(self.x, self.y)
                d2 = (new[:, 0] - mr) ** 2 + (new[:, 1] - mc) ** 2
                new = new[np.argsort(d2)[:WALL_SHARE_MAX]]
            cells = []
            for r, c in new:
                self._co4_known[r, c] = True
                x, y = self._c2w(int(r), int(c))
                cells.append([int(round(x * 10.0)), int(round(y * 10.0))])
            self._squad_tx.send(json.dumps(
                {"type": "walls", "id": self.robot_id, "c": cells}).encode())
        except Exception:
            pass

    def _co4_take_walls(self, msg):
        """co4: fold the partner's wall cells into the planning grid."""
        try:
            if msg.get("id") == self.robot_id or self._nav_wall is None:
                return
            cells = msg.get("c")
            if not isinstance(cells, list):
                return
            wall, n = self._nav_wall, self._nav_N
            if self._co4_known is None or self._co4_known.shape != wall.shape:
                self._co4_known = wall.astype(bool)
            added = 0
            for cell in cells[:4 * WALL_SHARE_MAX]:
                try:
                    x, y = float(cell[0]) / 10.0, float(cell[1]) / 10.0
                except Exception:
                    continue
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                if math.hypot(x - self.x, y - self.y) < WALL_SELF_CLEAR_M:
                    continue
                r, c = self._w2c(x, y)
                if not (0 <= r < n and 0 <= c < n):
                    continue
                self._co4_known[r, c] = True       # never echo it back
                if not wall[r, c]:
                    wall[r, c] = 1
                    added += 1
            if added:
                self._cost_dirty = True
                if self.robot.getTime() - self._last_plan_time > 1.5:
                    self._path = None
        except Exception:
            pass

    def _broadcast_victim(self, x, y):
''')

# send: once per loop after this robot's own lidar update (rate-limited inside)
rep("            self._update_map_from_lidar()\n",
    "            self._update_map_from_lidar()\n"
    "            self._co4_share_walls()\n")

# receive
rep('                elif msg.get("type") == "fail":\n',
    '                elif CO4 and msg.get("type") == "walls":\n'
    '                    self._co4_take_walls(msg)\n'
    '                elif msg.get("type") == "fail":\n')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
