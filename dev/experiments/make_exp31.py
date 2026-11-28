"""exp31: the partner false-positive zone applies only when the partner can
be in the picture. usage: make_exp31.py <in> <out>

b180 medium M0: both robots reached victim2 (0.7 and 0.8 m), each with
20+ candidate frames, and each was refused by "fp-zone" because the other
robot was within 2.5 m. The zone exists because the partner's chassis can
trigger the colour gate; that needs the partner inside the camera's field
of view. exp31 keeps the 2.5 m zone only when the partner's bearing is
within half the horizontal FOV plus 15 degrees of the heading (or within
0.8 m, where pose error makes the bearing unreliable).
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)
rep("""        return math.hypot(self.x - px, self.y - py) < 2.5
""", """        _d = math.hypot(self.x - px, self.y - py)
        if _d >= 2.5:
            return False
        if _d < 0.8:
            return True
        _b = math.atan2(py - self.y, px - self.x) - self.theta      # exp31
        _b = (_b + math.pi) % (2.0 * math.pi) - math.pi
        return abs(_b) < 0.52 + 0.26
""")
open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
