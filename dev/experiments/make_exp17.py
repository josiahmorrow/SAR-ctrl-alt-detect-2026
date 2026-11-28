"""exp17: reject flat, thin, untextured 'denim' at close range.
usage: make_exp17.py <in> <out>

Offline over 90 graded reports (exp07-nav3b, small+medium): every phantom
report (medium: plastic crate/cart, 3.3-3.7 m from the nearest victim, ~1 per
run) has saturation std inside the blob <= 6.5 AND vertical thickness
<= 0.052 m. Every TRUE report with tex_s < 7 has thickness >= 0.38 m (a limb
or torso); the one true-victim frame near the box (6.9 / 0.059) is a re-sight.
Physical reading: denim is a woven, shaded fabric on a body with depth;
moulded plastic is flat-shaded and, seen at the crate rim, a few cm tall.
Both cues must agree, and only inside 1.5 m where both are well measured.
SUSPECT-ISH: thresholds come from these two worlds' data, not first principles.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "FLAT_GATE           = True   # exp17: reject flat thin untextured blobs\n"
    "FLAT_TEX_S          = 6.8    # exp17: blob saturation std below this...\n"
    "FLAT_THICK_M        = 0.06   # exp17: ...and thinner than this\n"
    "FLAT_MAX_DEPTH_M    = 1.5    # exp17: only where both are well measured\n"
    "SECOND_LOOK         = True")

rep("""                except Exception:
                    thick_m = float('nan')
""", """                except Exception:
                    thick_m = float('nan')
                if (FLAT_GATE and min_depth < FLAT_MAX_DEPTH_M
                        and math.isfinite(tex_s) and math.isfinite(thick_m)
                        and tex_s < FLAT_TEX_S and thick_m < FLAT_THICK_M):
                    return self._vision_reject("flat-plastic", blob=blob_frac,
                                               ratio=ratio, depth=min_depth)
""")
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
