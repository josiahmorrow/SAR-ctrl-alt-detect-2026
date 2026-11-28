"""Apply exp06 (reposition ends at the body's edge) to a controller file.
usage: python make_exp06.py <in.py> <out.py>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()

a_old = "RECOVER_PULSE_S    = 0.6"
a_new = ("REPO_BODY_IR_M     = 0.15   # exp06: low obstacle this close during a\n"
         "                            # reposition = the victim's body; stop here\n"
         + a_old)
assert src.count(a_old) == 1
src = src.replace(a_old, a_new)

b_old = """            elif now - self._repo_stall_t > 2.5:
                stalled = True                    # furniture in the way — report here
"""
b_new = b_old + """            # exp06: a lying body is a low obstacle the lidar cannot see, and
            # the IR guard is off while closing on a victim. When the front IR
            # finds it right ahead, driving on means climbing the victim
            # (small seed 1: 160 s high-centred, odometry 40 m out). The body's
            # edge is the closest reachable point toward the estimate.
            if self._ir_front_m() < REPO_BODY_IR_M:
                stalled = True
"""
assert src.count(b_old) == 1, src.count(b_old)
src = src.replace(b_old, b_new)
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
