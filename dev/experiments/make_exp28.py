"""exp28: refuse to report a flat-shaded (untextured) surface at report range.
usage: make_exp28.py <in> <out>

Physical basis: denim is a woven fabric; at report range (<= ~0.8 m) its
weave puts high-frequency luminance variation into the image. A painted or
moulded surface (the medium crate cart, 8 of 12 kept medium runs) is flat
shaded: smooth gradients, no fine detail. Measure: mean |V - blur5(V)| over
the eroded denim blob, divided by mean V (scale and exposure free).
Offline on snapshot frames (texcheck.py, snapM3 + snapS6):
  crate at report range 0.0047-0.0070 (0.0072-0.0094 at 1.0-1.1 m);
  true victims at <= 0.9 m 0.0103-0.0334 (lowest: small v4, partly occluded).
Cut FINE_HP_MIN = 0.0085, applied only once the report streak completes
(depth <= APPROACH_STOP_M), only when the eroded blob has >= 200 px.
A rejected surface's projected position (pose + (depth+0.3) along heading,
as REPO_MATCH) blocks new homing within FLAT_SPOT_R_M for the rest of the
mission, so the robot does not re-home on it from another angle.
"""
import sys
src = open(sys.argv[1], encoding="utf-8").read()


def rep(old, new, n=1):
    global src
    c = src.count(old)
    assert c == n, (c, old[:80])
    src = src.replace(old, new)


rep("HOME_BLOCK_R_M      = 1.5\n",
    "HOME_BLOCK_R_M      = 1.5\n"
    "FINE_GATE           = True    # exp28: refuse flat-shaded surfaces\n"
    "FINE_HP_MIN         = 0.0085  # exp28: high-pass energy / mean V\n"
    "FLAT_SPOT_R_M       = 1.0     # exp28: no re-homing near a rejected one\n")

rep("        self._reported_positions = []\n",
    "        self._reported_positions = []\n"
    "        self._flat_spots       = []      # exp28\n")

# measure in the vision pipeline
rep("""        except Exception:
            tex_v = tex_s = float('nan')
""", """        except Exception:
            tex_v = tex_s = float('nan')
        hp = float('nan')                     # exp28: fine texture
        try:
            _er = cv2.erode(blob, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (7, 7))).astype(bool)
            if int(_er.sum()) >= 200:
                _V = hsv[:, :, 2].astype(np.float32)
                _mv = float(_V[_er].mean())
                hp = float(np.abs(_V - cv2.GaussianBlur(_V, (5, 5), 0))[_er].mean()) \\
                    / max(_mv, 1.0)
        except Exception:
            hp = float('nan')
""")
rep('''                             "tex_v": tex_v, "tex_s": tex_s, "thick_m": thick_m,''',
    '''                             "tex_v": tex_v, "tex_s": tex_s, "thick_m": thick_m,
                             "hp": hp,''')

# gate at report time (main branch, once the streak is complete)
rep('''            self.stop()
            # Marker-offset countermeasure: the CSV estimate is a better proxy
''', '''            self.stop()
            if FINE_GATE:
                _hp = (self._last_vision or {}).get("hp", float("nan"))
                if math.isfinite(_hp) and _hp < FINE_HP_MIN:
                    _fx = self.x + (depth + 0.3) * math.cos(self.theta)
                    _fy = self.y + (depth + 0.3) * math.sin(self.theta)
                    self._flat_spots.append((_fx, _fy))
                    print(f"[{self.robot_id}] exp28 flat surface (hp={_hp:.4f}) "
                          f"at ({_fx:.2f},{_fy:.2f}) - not reporting")
                    self._end_homing(advance=False)
                    return
            # Marker-offset countermeasure: the CSV estimate is a better proxy
''')

# no new homing on a rejected flat spot
rep('''        conf, depth, _ = self._camera_victim_confidence()
        # Require a FINITE denim depth in range: a denim patch with no depth''',
    '''        conf, depth, _ = self._camera_victim_confidence()
        if FINE_GATE and self._flat_spots and math.isfinite(depth):
            _fx = self.x + (depth + 0.3) * math.cos(self.theta)
            _fy = self.y + (depth + 0.3) * math.sin(self.theta)
            if any(math.hypot(_fx - sx, _fy - sy) < FLAT_SPOT_R_M
                   for sx, sy in self._flat_spots):
                self._home_block = "flat-spot"
                return
        # Require a FINITE denim depth in range: a denim patch with no depth''')

open(sys.argv[2], "w", encoding="utf-8").write(src)
print("ok")
