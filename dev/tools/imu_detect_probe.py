"""Probe flyover_extraction's IMU-assisted victim path, and its sensitivity to
tiny decoder differences.

For each world: raw IMU-projected point count, which clustering regime that
count selects (the dense short-route regime switches at
IMU_DENSE_SHORT_ROUTE_MIN_PTS), and the resulting estimates vs ground truth.
With --perturb, the same is repeated with every decoded pixel shifted by +/-1
grey level, i.e. the size of an ordinary cross-platform YUV->BGR rounding
difference.

    python dev/tools/imu_detect_probe.py [small medium large] [--perturb]
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_pristine import REPO, SOL          # noqa: E402
import eval_harness as EH                     # noqa: E402

sys.path.insert(0, SOL)
import flyover_extraction as fx               # noqa: E402

_RealCap = cv2.VideoCapture


def capture_with_offset(delta):
    class Cap(_RealCap):
        def read(self, *a):
            ok, fr = super().read(*a)
            if ok and delta:
                fr = cv2.add(fr, np.full_like(fr, delta)) if delta > 0 else \
                    cv2.subtract(fr, np.full_like(fr, -delta))
            return ok, fr
    return Cap


def gt(world):
    W = EH.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    o = W["origin"]
    return [(v["marker_world"][0] - o[0], v["marker_world"][1] - o[1])
            for v in W["victims"]]


def probe(world, delta):
    fx.cv2.VideoCapture = capture_with_offset(delta)
    try:
        vid = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
        imu = os.path.splitext(vid)[0] + ".csv"
        captured = {}
        orig = fx.cluster_detections_with_params

        def spy(pts, *a, **k):
            captured["n"] = len(pts)
            captured["params"] = a
            return orig(pts, *a, **k)
        fx.cluster_detections_with_params = spy
        est = fx.detect_victims_with_imu_projection(vid, imu)
        fx.cluster_detections_with_params = orig
    finally:
        fx.cv2.VideoCapture = _RealCap
    cap = cv2.VideoCapture(vid)
    dur = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    dense = (captured["n"] >= fx.IMU_DENSE_SHORT_ROUTE_MIN_PTS
             and dur < fx.IMU_LONG_ROUTE_DURATION_S)
    g = gt(world)
    errs = [min(math.hypot(x - a, y - b) for x, y in est) if est else float("inf")
            for a, b in g]
    print(f"{world:6} delta={delta:+d}  raw={captured['n']:5d}  "
          f"regime={'DENSE' if dense else 'normal'}  estimates={len(est)}  "
          f"GT->nearest est: " + " ".join(f"{e:4.2f}" for e in errs))


if __name__ == "__main__":
    worlds = [a for a in sys.argv[1:] if not a.startswith("--")] or \
        ["small", "medium", "large"]
    deltas = (-1, 0, 1) if "--perturb" in sys.argv else (0,)
    for w in worlds:
        for d in deltas:
            probe(w, d)
