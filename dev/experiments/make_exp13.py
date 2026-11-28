"""exp13: gravity-guided recovery from high-centring. usage: make_exp13.py <in> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("RECOVER_PULSE_S    = 0.6",
    "RECOVER_GRAVITY    = True   # exp13: drive downhill (accelerometer) to get off\n"
    "RECOVER_SETTLE_S   = 0.6    # exp13: stopped, so the accelerometer reads gravity\n"
    "RECOVER_PUSH_S     = 2.0    # exp13: sustained push downhill\n"
    "RECOVER_PULSE_S    = 0.6")

rep("""                cycle = (now_g - self._recover_start) % (RECOVER_MAX_S
                                                         + RECOVER_REST_S)
                if cycle < RECOVER_MAX_S:""",
"""                if RECOVER_GRAVITY:
                    self._recover_gravity_step(now_g)
                    continue
                cycle = (now_g - self._recover_start) % (RECOVER_MAX_S
                                                         + RECOVER_REST_S)
                if cycle < RECOVER_MAX_S:""")

rep("""    def _read_tilt(self):""",
'''    def _recover_gravity_step(self, now):
        """exp13: high-centred on a body or debris (25-80 deg), the symmetric
        fwd/rev wiggle cancels itself and never frees the chassis (small:
        robots sat at 25 deg for the rest of the mission after a report). Stop
        long enough for the accelerometer to read gravity alone, then push
        hard toward the downhill side of the chassis: nose up (x component of
        the measured specific force > 0) means downhill is behind us. Every
        third push goes the other way in case the low side is blocked."""
        st = self.__dict__.setdefault("_rg", {"phase": "settle", "t": now,
                                              "dir": -1.0, "n": 0})
        if st["phase"] == "settle":
            self.stop()
            if now - st["t"] >= RECOVER_SETTLE_S and self._accel:
                try:
                    ax, ay, az = self._accel.getValues()
                except Exception:
                    ax, ay = 1.0, 0.0
                d = -1.0 if ax > 0 else 1.0           # downhill along the body axis
                st["n"] += 1
                if st["n"] % 3 == 0:
                    d = -d
                # mostly rolled: turn so the low side leads
                st["turn"] = 0.0
                if abs(ay) > 1.5 * abs(ax):
                    st["turn"] = -0.5 if ay > 0 else 0.5
                st.update(phase="push", t=now, dir=d)
                if DEBUG_VISION:
                    print("[%s] t=%6.1f RECOVER push dir=%+.0f turn=%+.1f acc=(%.2f,%.2f)"
                          % (self.robot_id, now, d, st["turn"], ax, ay))
            return
        if now - st["t"] < RECOVER_PUSH_S:
            v = MAX_SPEED * 0.9 * st["dir"]
            t = MAX_SPEED * 0.4 * st["turn"]
            self.set_speeds(v - t, v + t)
            return
        st.update(phase="settle", t=now)

    def _read_tilt(self):''')

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
