"""exp14: shrink the partner false-positive zone. usage: make_exp14.py <in> <out>"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()
old = """        return math.hypot(self.x - px, self.y - py) < 2.5"""
new = """        # exp14: 2.5 m was set before the denim gate existed, when the
        # partner's red chassis passed the colour test. A red chassis cannot
        # pass the denim gate, and 2.5 m round a partner beached beside a
        # victim hides that victim from both robots for the rest of the run
        # (nav2 small s0: 312 candidate frames blocked by fp-zone). Keep only
        # a contact-range zone.
        return math.hypot(self.x - px, self.y - py) < PARTNER_FP_R_M"""
assert src.count(old) == 1
src = src.replace(old, new)
old2 = "SECOND_LOOK         = True"
assert src.count(old2) == 1
src = src.replace(old2, "PARTNER_FP_R_M      = 0.8    # exp14: was 2.5\n" + old2)
open(sys.argv[2], "w", encoding="utf-8", newline="").write(src)
print("ok")
