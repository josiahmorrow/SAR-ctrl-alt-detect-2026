"""Is cv2.VideoCapture seeking (CAP_PROP_POS_FRAMES) frame-exact on this platform?

The flyover pipelines pair video frame fi with IMU row fi (t = fi / fps). If a
seek lands on a different frame than requested, every pose built from that
frame is paired with the wrong IMU sample. This compares seeked frames with
the same index reached by sequential decoding.

    python dev/tools/seek_check.py recordings/large_world_flyover.mp4
"""
import sys

import cv2
import numpy as np

path = sys.argv[1]
probe = [int(x) for x in sys.argv[2:]] or [62, 500, 1250, 2500, 3750, 4700]
cap = cv2.VideoCapture(path)
print("backend", cap.getBackendName(), "frames", int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
      "fps", cap.get(cv2.CAP_PROP_FPS))
seq = {}
i = 0
want = set(probe) | {p + d for p in probe for d in range(-40, 41)}
while True:
    ok, fr = cap.read()
    if not ok:
        break
    if i in want:
        seq[i] = cv2.resize(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), (160, 90)).astype(np.float32)
    i += 1
for fi in probe:
    c = cv2.VideoCapture(path)
    c.set(cv2.CAP_PROP_POS_FRAMES, fi)
    reported = int(c.get(cv2.CAP_PROP_POS_FRAMES))
    ok, fr = c.read()
    g = cv2.resize(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), (160, 90)).astype(np.float32)
    errs = {k: float(np.mean(np.abs(v - g))) for k, v in seq.items() if abs(k - fi) <= 40}
    best = min(errs, key=errs.get)
    print(f"seek {fi:5d}: pos-after-set {reported:5d}  best match seq frame {best:5d} "
          f"(offset {best - fi:+d}, err {errs[best]:.2f}; err at requested {errs.get(fi, -1):.2f})")
