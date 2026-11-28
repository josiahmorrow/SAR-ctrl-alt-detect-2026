"""exp25 (on the kept stack): flank a body that extends AWAY from the robot,
judged by the depth spread of its denim, not by its apparent width.
usage: make_exp25.py <in ps_kept.py> <out>

The scorer measures the robot's true position against the victim's node
(supervisor _check_victim_proximity, radius 1.0 m). Head-on, the camera
loses the denim below ~0.62 m, so the robot cannot park closer than that to
the body's near edge. Broadside the node is ~0.2 m behind the near edge and
the report scores; when the body extends away from the robot (end-on or
oblique) the node is 0.5-1.2 m behind it and the report lands at 1.04-1.31 m
(12 kept-stack runs: medium victim3 four times at 1.15-1.21 m, small five
times). exp18 tried to detect this from the blob's width and fired once in
12 runs: oblique bodies still look 0.55-0.67 m wide. The physical quantity
is the body's extent ALONG the line of sight: the spread of the denim depth
(p90 - p10). When it exceeds FLANK_EXT_M, use exp18's flank (step 0.6 m
sideways to the roomier side, drive abreast of the denim's far half,
report there).
"""
import subprocess, sys
D = r"C:\Users\agcoo\projects\sar\_scratch"
subprocess.run([sys.executable, rf"{D}\make_exp18k.py", sys.argv[1], sys.argv[2] + ".tmp"], check=True)
src = open(sys.argv[2] + ".tmp", encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("FLANK_TIMEOUT_S     = 9.0    # exp18\n",
    "FLANK_TIMEOUT_S     = 9.0    # exp18\n"
    "FLANK_EXT_M         = 0.35   # exp25: denim depth spread (p90-p10) => body runs away\n")

# measure the depth spread of the denim blob
rep("""                d, rows = d[finite], rows[finite]
                # MEDIAN, not min""",
    """                d, rows = d[finite], rows[finite]
                try:
                    d_ext = float(np.percentile(d, 90) - np.percentile(d, 10))
                except Exception:
                    d_ext = float('nan')
                # MEDIAN, not min""")
rep("""        width_m = float('nan')
        thick_m = float('nan')""",
    """        width_m = float('nan')
        thick_m = float('nan')
        d_ext   = float('nan')""")
rep('''                             "width_m": width_m''',
    '''                             "width_m": width_m, "d_ext": d_ext''')

# trigger on the depth spread as well as the narrow-blob case
rep("""                if math.isfinite(_wm) and _wm < FLANK_END_ON_W_M:""",
    """                _ext = (self._last_vision or {}).get("d_ext", float("nan"))
                if ((math.isfinite(_wm) and _wm < FLANK_END_ON_W_M)
                        or (math.isfinite(_ext) and _ext >= FLANK_EXT_M)):""")
rep("""                        print(f"[{self.robot_id}] End-on body (w={_wm:.2f} m) - \"""",
    """                        print(f"[{self.robot_id}] Body runs away (w={_wm:.2f} m, "
                              f"depth spread {_ext:.2f} m) - \"""")

# telemetry: depth spread at every report
rep("""        print(f"[{self.robot_id}] Victim reported @ ({self.x:.2f}, {self.y:.2f})  "
              f"conf={HIGH_CONFIDENCE:.2f}")""",
    """        _v = self._last_vision or {}
        print(f"[{self.robot_id}] Victim reported @ ({self.x:.2f}, {self.y:.2f})  "
              f"conf={HIGH_CONFIDENCE:.2f} depth={_v.get('depth', float('nan')):.2f} "
              f"wid={_v.get('width_m', float('nan')):.2f} "
              f"ext={_v.get('d_ext', float('nan')):.2f}")""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
import os
os.remove(sys.argv[2] + ".tmp")
print("ok")
