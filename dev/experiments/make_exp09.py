"""exp09: brake for low obstacles the front IR sees (speed governor).
usage: python make_exp09.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

def rep(old, new):
    global src
    n = src.count(old)
    assert n == 1, (n, old[:70])
    src = src.replace(old, new)

rep("IR_CLEAR_V         = 1.9",
    "IR_SLOW_M          = 0.9    # exp09: front IR closer than this -> slow down\n"
    "IR_SLOW_MIN        = 0.3    # exp09: floor of the speed scale\n"
    "IR_CLEAR_V         = 1.9")

rep("""        turn  = MAX_SPEED * 0.45 * heading_error
        speed = MAX_SPEED * 0.8
        front = self._min_front_range()""",
"""        turn  = MAX_SPEED * 0.45 * heading_error
        speed = MAX_SPEED * 0.8
        # exp09: something low ahead (below the lidar plane) -> arrive at it
        # slowly. Large: both robots flipped on first contact with rubble the
        # IR had seen 0.2-0.4 m out at cruise speed (accelerometer spikes of
        # 6-20 g); contact impulse scales with closing speed.
        if IR_SLOW_M > 0:
            _irs = self._ir_front_m()
            if _irs < IR_SLOW_M:
                speed *= max(IR_SLOW_MIN, (_irs - 0.15) / (IR_SLOW_M - 0.15))
        front = self._min_front_range()""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
