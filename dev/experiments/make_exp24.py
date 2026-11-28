"""exp24: escalate a failed straight reverse in the 11-25 deg band to the
rocking recovery. usage: make_exp24.py <in> <out>

Below TILT_RECOVER_LO (25 deg) the tilt guard holds a straight reverse for
as long as the chassis stays tilted. When the reverse frees the robot it
does so within a second or two; robots still tilted after that stay so for
the rest of the run (beach.py: tail tilt 13-25 deg for 200-450 s in exp16,
exp20, exp23 runs). A straight reverse that has not worked in
STUCK_WIGGLE_AFTER_S will not; switch to the same fwd/back/turn pulse cycle
used above 25 deg, which changes which wheels bear load.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "STUCK_WIGGLE        = True   # exp24: escalate a failed straight reverse\n"
    "STUCK_WIGGLE_AFTER_S = 4.0   # exp24\n"
    "SECOND_LOOK         = True")

rep("""            if now_g < self._tilt_escape_until:
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                continue
""", """            if now_g < self._tilt_escape_until:
                if (STUCK_WIGGLE and self._tilt_hi_since is not None
                        and now_g - self._tilt_hi_since > STUCK_WIGGLE_AFTER_S):
                    _cyc = ((now_g - self._tilt_hi_since - STUCK_WIGGLE_AFTER_S)
                            % (RECOVER_MAX_S + RECOVER_REST_S))
                    if _cyc < RECOVER_MAX_S:
                        _ph = int(_cyc / RECOVER_PULSE_S) % 4
                        _vl, _vr = ((-0.45, -0.45), (0.45, 0.45),
                                    (-0.2, 0.5), (0.5, -0.2))[_ph]
                        self.set_speeds(MAX_SPEED * _vl, MAX_SPEED * _vr)
                    else:
                        self.stop()
                    continue
                self.set_speeds(-MAX_SPEED * 0.5, -MAX_SPEED * 0.5)
                continue
""")
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
