"""exp26 (on kept exp24): accept a smaller visible denim area where the
flyover already says a victim lies. usage: make_exp26.py <in> <out>

missed.py over the 12 kept-stack small runs: small v4 is missed in 6 of 12,
and in those runs a robot stood 0.7-1.9 m from it for up to 438 s with no
candidate frame. Its denim WAS seen: every frame was rejected
'denim-area-small', visible area 0.026-0.031 m2 at floor height (h_med
0.06-0.08 m), against VICTIM_DENIM_MIN_AREA_M2 = 0.032 (set between a small
blue decoy, 0.028-0.030 m2, and an occluded victim, 0.037 m2). Area alone
cannot separate them. The flyover estimate is independent evidence: within
EST_NEAR_AREA_M of an unreported estimate, the area gate drops to
VICTIM_DENIM_MIN_AREA_EST_M2; everywhere else it is unchanged. Flagged as a
threshold change (rule 7): the justification is prior evidence, not a fit.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("SECOND_LOOK         = True",
    "AREA_NEAR_EST       = True   # exp26\n"
    "EST_NEAR_AREA_M     = 3.0    # exp26: robot this close to an unreported estimate\n"
    "VICTIM_DENIM_MIN_AREA_EST_M2 = 0.022  # exp26: area gate there\n"
    "SECOND_LOOK         = True")

rep("""                if area_m2 < VICTIM_DENIM_MIN_AREA_M2:""",
    """                _thr = VICTIM_DENIM_MIN_AREA_M2
                if AREA_NEAR_EST and area_m2 >= VICTIM_DENIM_MIN_AREA_EST_M2:
                    try:
                        _e, _de = self._nearest_estimate()
                        if _e is not None and _de <= EST_NEAR_AREA_M:
                            _thr = VICTIM_DENIM_MIN_AREA_EST_M2
                    except Exception:
                        pass
                if area_m2 < _thr:""")

open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
