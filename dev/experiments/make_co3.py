"""co3: share sightings a robot cannot turn into a report.
usage: make_co3.py <in> <out>

A robot that sees a solid candidate but cannot act on it tells nobody.
The clearest case is the 8 s cooldown after a FAILED homing (lost, stalled
against something, timed out): the body is still in view, the robot has
just shown it cannot get to it from here, and the partner may be able to
from its side.

co3 sends a new channel-73 message while that is the case:
    {"type": "sight", "id": "<robot id>", "x": <m>, "y": <m>}
(x, y) is the candidate's ground position (pose + (depth + 0.3) along the
camera bearing). Sent at most once per SIGHT_EVERY_S, never for a point
within SIGHT_SKIP_REPORTED_M of a confirmed report (it would be the body
already reported), never twice for the same spot (SIGHT_SAME_M).

The receiver adds the point as one more waypoint at the END of its flyover
leg (so its own estimates come first), unless it is near a confirmed
report, near a waypoint it already has or has abandoned, in a blacklisted
region, or it has already taken SIGHT_MAX_ADOPT sightings.
An unpatched partner ignores the unknown message type; a silent partner
changes nothing.
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
    "CO3                 = True   # co3: share sightings that cannot be reported\n"
    "SIGHT_EVERY_S       = 5.0    # co3\n"
    "SIGHT_SAME_M        = 1.0    # co3: one sighting per spot\n"
    "SIGHT_SKIP_REPORTED_M = 1.5  # co3: that is the body already reported\n"
    "SIGHT_MAX_ADOPT     = 3      # co3\n")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._co3_sent         = []      # co3: sightings broadcast\n"
    "        self._co3_next_t       = 0.0     # co3\n"
    "        self._co3_adopted      = []      # co3: sightings taken from the partner\n")

rep("    def _broadcast_victim(self, x, y):\n", '''    def _co3_share_sighting(self):
        """co3: broadcast the candidate in view (called when homing on it is
        refused by the post-failure cooldown)."""
        if not CO3 or not self._squad_tx:
            return
        now = self.robot.getTime()
        if now < self._co3_next_t:
            return
        v = self._last_vision or {}
        d = v.get("depth", float("inf"))
        if v.get("why") != "candidate" or not isinstance(d, (int, float)) \\
           or not math.isfinite(d) or d > APPROACH_ENTER_M \\
           or (v.get("conf", 0.0) or 0.0) < APPROACH_ENTER_CONF:
            return
        try:
            hf = float(self._camera.getFov())
        except Exception:
            hf = 1.0
        xo = v.get("x_offset", 0.0) or 0.0
        th = self.theta - math.atan(xo * math.tan(hf * 0.5))
        sx = self.x + (d + 0.3) * math.cos(th)
        sy = self.y + (d + 0.3) * math.sin(th)
        if any(math.hypot(sx - rx, sy - ry) < SIGHT_SKIP_REPORTED_M
               for rx, ry in self._reported_positions):
            return
        if any(math.hypot(sx - px, sy - py) < SIGHT_SAME_M
               for px, py in self._co3_sent):
            return
        self._co3_next_t = now + SIGHT_EVERY_S
        self._co3_sent.append((sx, sy))
        try:
            self._squad_tx.send(json.dumps(
                {"type": "sight", "id": self.robot_id,
                 "x": round(sx, 2), "y": round(sy, 2)}).encode())
            print(f"[{self.robot_id}] co3 sighting shared ({sx:.2f},{sy:.2f})")
        except Exception:
            pass

    def _co3_take_sighting(self, msg):
        """co3: maybe add the partner's sighting to this robot's flyover leg."""
        try:
            if msg.get("id") == self.robot_id:
                return                             # our own broadcast
            sx, sy = float(msg["x"]), float(msg["y"])
        except Exception:
            return
        if not (math.isfinite(sx) and math.isfinite(sy)):
            return
        if len(self._co3_adopted) >= SIGHT_MAX_ADOPT:
            return
        if any(math.hypot(sx - rx, sy - ry) < SIGHT_SKIP_REPORTED_M
               for rx, ry in self._reported_positions):
            return
        if any(math.hypot(sx - wx, sy - wy) < SIGHT_SAME_M
               for wx, wy in self._my_waypoints[:self._n_video_wps]):
            return
        if any(math.hypot(sx - fx, sy - fy) < 1.5
               for fx, fy in getattr(self, "_failed_wps", [])):
            return
        if any(math.hypot(sx - ax, sy - ay) < SIGHT_SAME_M
               for ax, ay in self._co3_adopted):
            return
        try:
            if self._region_blacklisted(sx, sy):
                return
        except Exception:
            pass
        self._co3_adopted.append((sx, sy))
        self._my_waypoints.insert(self._n_video_wps, (sx, sy))
        self._n_video_wps += 1
        print(f"[{self.robot_id}] co3 partner sighting queued ({sx:.2f},{sy:.2f})")

    def _broadcast_victim(self, x, y):
''')

# send: during the post-failure cooldown, with the body still in view
rep('''        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''', '''        if CO3 and self.robot.getTime() < self._home_cooldown_until:
            self._co3_share_sighting()
        if self.robot.getTime() < self._home_cooldown_until:
            self._home_block = "cooldown"
            return
''')

# receive
rep('                elif msg.get("type") == "fail":\n',
    '                elif CO3 and msg.get("type") == "sight":\n'
    '                    self._co3_take_sighting(msg)\n'
    '                elif msg.get("type") == "fail":\n')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
