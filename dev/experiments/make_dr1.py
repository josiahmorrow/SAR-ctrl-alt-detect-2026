"""dr1: victim estimates from flyover_dr (body-frame dead reckoning) replace
the vendored IMU-projection estimates in the graded CSV / robot waypoints.
The vendored pipeline still runs for the wall map and as the fallback.
usage: make_dr1.py <in> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)
rep('''        v, m = fx.analyse(mp4, imu)
        fx.save(v, m)
''', '''        v, m = fx.analyse(mp4, imu)
        fx.save(v, m)
        # dr1: replace the victim estimates with the dead-reckoned ones. The
        # vendored estimates stay in place if this fails for any reason.
        try:
            import flyover_dr
            _imu = imu or (os.path.splitext(mp4)[0] + ".csv")
            _dv = flyover_dr.locate_victims(mp4, _imu, fx)
            if _dv:
                with open(fx.CSV_OUT, "w") as _f:
                    for _x, _y, _n in _dv:
                        _f.write(f"{_x:.6f},{_y:.6f}\\n")
                print(f"[preflight] victim estimates from flyover_dr: {len(_dv)}")
            else:
                print("[preflight] flyover_dr found nothing - keeping the "
                      "vendored estimates")
        except Exception as e:
            print(f"[preflight] WARNING - flyover_dr failed ({e}) - keeping "
                  f"the vendored estimates")
''')
open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
