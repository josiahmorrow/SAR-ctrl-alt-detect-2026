"""exp19: reposition toward a flyover estimate only if it is THIS body's.
usage: make_exp19.py <in> <out>

The marker-offset countermeasure drives from the locked denim to the nearest
unreported flyover estimate 0.9-2.2 m away and reports there. When that
estimate belongs to the body in view it helps; when it does not, the robot
drives away from the victim and reports from 1.3-1.6 m (medium victim2 in
exp08b M3/M4, exp16 M0/M3/M4, exp18 M3/M4: 'Denim locked - repositioning
toward estimate (10.81,-24.86)' while the body lay the other way).
Geometric test: the body's centroid is ahead along the camera axis at about
depth + 0.3 m; the estimate must lie within REPO_MATCH_M of it.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "REPO_MATCH          = True   # exp19: estimate must match the body in view\n"
    "REPO_MATCH_M        = 1.0    # exp19\n"
    "SECOND_LOOK         = True")

rep("""            est, d_est = self._nearest_estimate()
            if est is not None and REPOSITION_MIN_M < d_est <= REPOSITION_MAX_M:""",
    """            est, d_est = self._nearest_estimate()
            if est is not None and REPO_MATCH:
                _bx = self.x + (depth + 0.3) * math.cos(self.theta)
                _by = self.y + (depth + 0.3) * math.sin(self.theta)
                if math.hypot(est[0] - _bx, est[1] - _by) > REPO_MATCH_M:
                    print(f"[{self.robot_id}] Estimate ({est[0]:.2f},{est[1]:.2f}) "
                          f"is not this body's - reporting from here")
                    est = None
            if est is not None and REPOSITION_MIN_M < d_est <= REPOSITION_MAX_M:""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
