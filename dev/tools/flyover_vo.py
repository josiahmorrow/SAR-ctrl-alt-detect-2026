"""Prototype: victim localisation from the flyover video by visual tracking.

No IMU, no duration-keyed constants. The origin marker (ArUco, known size)
fixes scale and frame while it is visible; after that the pixel->world
homography is carried frame to frame by a RANSAC similarity fitted to
optical-flow tracks (the floor is the dominant plane under a downward
camera). Denim blobs are projected through the current homography and
clustered on the ground.

    python dev/tools/flyover_vo.py small medium large [--step 4] [--dump]

Prints the estimates and, OFFLINE ONLY, their error against the world file
(eval_harness). Nothing here is used by the controller yet.
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SOL = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, SOL)
sys.path.insert(0, HERE)
import flyover_extraction as fx          # noqa: E402  (ArUco calibration only)

REPO = os.path.dirname(os.path.dirname(SOL))
SMALL = 0.5                               # tracking resolution factor
S = np.diag([SMALL, SMALL, 1.0])
S_INV = np.diag([1.0 / SMALL, 1.0 / SMALL, 1.0])
DENIM_LO, DENIM_HI = np.array([95, 35, 25]), np.array([135, 255, 255])
AREA_MIN_M2, AREA_MAX_M2 = 0.03, 0.80     # visible denim of a lying person
CLUSTER_R_M, MIN_VOTES = 1.0, 6
RIGID = "--rigid" in sys.argv
MOSAIC = "--mosaic" in sys.argv
FLOOR = "--floor" in sys.argv
MOS = {}


def m_per_px(H, px, py):
    """Local ground scale of homography H at a pixel (metres per pixel)."""
    p0 = np.array(fx.pix_to_world(px, py, H))
    p1 = np.array(fx.pix_to_world(px + 10.0, py, H))
    p2 = np.array(fx.pix_to_world(px, py + 10.0, H))
    return 0.05 * (np.linalg.norm(p1 - p0) + np.linalg.norm(p2 - p0))


def track(prev_g, cur_g):
    """Similarity cur(small px) -> prev(small px), or None."""
    p0 = cv2.goodFeaturesToTrack(prev_g, 800, 0.01, 10)
    if p0 is None or len(p0) < 30:
        return None, 0
    p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_g, cur_g, p0, None,
                                         winSize=(21, 21), maxLevel=3)
    ok = st.ravel() == 1
    if ok.sum() < 30:
        return None, 0
    A, inl = cv2.estimateAffinePartial2D(p1[ok], p0[ok], method=cv2.RANSAC,
                                         ransacReprojThreshold=1.5)
    if A is None:
        return None, 0
    if FLOOR:
        # Under a downward camera the floor is the farthest surface, so it
        # has the SMALLEST image motion; table tops and wall tops move
        # faster. Peel up to three motion layers and keep the slowest one
        # that still has real support, instead of the most numerous.
        a1, a0 = p1[ok].reshape(-1, 2), p0[ok].reshape(-1, 2)
        n_all = len(a1)
        layers = []
        for _ in range(3):
            if len(a1) < 20:
                break
            Ak, ik = cv2.estimateAffinePartial2D(a1, a0, method=cv2.RANSAC,
                                                 ransacReprojThreshold=1.5)
            if Ak is None:
                break
            ik = ik.ravel() == 1
            if ik.sum() < max(15, 0.12 * n_all):
                break
            ctr = np.array([cur_g.shape[1] / 2.0, cur_g.shape[0] / 2.0])
            mv = np.linalg.norm(Ak[:, :2] @ ctr + Ak[:, 2] - ctr)
            layers.append((mv, Ak, a1[ik], a0[ik]))
            a1, a0 = a1[~ik], a0[~ik]
        if layers:
            mv, A, q1f, q0f = min(layers, key=lambda L: L[0])
            ang = np.arctan2(A[1, 0], A[0, 0])
            R = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
            t = np.median(q0f - q1f @ R.T, axis=0)
            return np.vstack([np.hstack([R, t.reshape(2, 1)]), [0, 0, 1]]), len(q1f)
    if RIGID:
        # Level flight at constant height: rotation + translation only. The
        # similarity's scale term is biased by parallax on anything above the
        # floor and its error compounds over a thousand frames.
        ang = np.arctan2(A[1, 0], A[0, 0])
        R = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        q1 = p1[ok].reshape(-1, 2)[inl.ravel() == 1]
        q0 = p0[ok].reshape(-1, 2)[inl.ravel() == 1]
        t = np.median(q0 - q1 @ R.T, axis=0)
        A = np.hstack([R, t.reshape(2, 1)])
    return np.vstack([A, [0, 0, 1]]), int(inl.sum())


def denim_blobs(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, DENIM_LO, DENIM_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, _, stats, cents = cv2.connectedComponentsWithStats(m, 8)
    return [(cents[i][0], cents[i][1], stats[i, cv2.CC_STAT_AREA])
            for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 150]


def cluster(pts):
    pts = list(pts)
    out = []
    while pts:
        best = max(pts, key=lambda p: sum(
            1 for q in pts if np.hypot(p[0] - q[0], p[1] - q[1]) < CLUSTER_R_M))
        mem = [q for q in pts if np.hypot(best[0] - q[0], best[1] - q[1]) < CLUSTER_R_M]
        if len(mem) < MIN_VOTES:
            break
        out.append((float(np.median([q[0] for q in mem])),
                    float(np.median([q[1] for q in mem])), len(mem)))
        pts = [q for q in pts
               if np.hypot(best[0] - q[0], best[1] - q[1]) >= CLUSTER_R_M * 1.5]
    return out


def run(world, step=4, dump=False):
    cap = cv2.VideoCapture(os.path.join(REPO, "recordings",
                                        f"{world}_world_flyover.mp4"))
    fps = cap.get(cv2.CAP_PROP_FPS)
    H = None                 # full-res pixel -> world, current frame
    prev_g = None
    pts, path = [], []
    fi = -1
    anchored = tracked = lost = 0
    while True:
        ok = cap.grab()
        if not ok:
            break
        fi += 1
        if fi % step:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        g = cv2.cvtColor(cv2.resize(frame, None, fx=SMALL, fy=SMALL,
                                    interpolation=cv2.INTER_AREA),
                         cv2.COLOR_BGR2GRAY)
        Ha = None
        mk = fx.detect_aruco(frame)
        if 0 in mk and len(mk) >= 3:
            Ha = fx.build_calibration_H(mk)
        if Ha is not None:
            # A level, downward camera sees the floor through a similarity.
            # The marker's own homography has noisy perspective terms that
            # explode when extrapolated metres away, so keep only the
            # similarity it implies at the marker.
            ci = np.float32([c for m_ in mk.values() for c in m_])
            cw = np.float32([fx.pix_to_world(c[0], c[1], Ha) for c in ci])
            # pixel->world includes a reflection (image y is down); a proper
            # similarity cannot hold one, so try both hands and keep the fit.
            best = None
            for F in (np.diag([1.0, 1.0, 1.0]), np.diag([1.0, -1.0, 1.0])):
                cf = np.float32(ci * np.float32([F[0, 0], F[1, 1]]))
                As, _ = cv2.estimateAffinePartial2D(cf, cw)
                if As is None:
                    continue
                Hc = np.vstack([As, [0, 0, 1]]) @ F
                err = np.mean([np.hypot(*(np.array(fx.pix_to_world(c[0], c[1], Hc)) - w_))
                               for c, w_ in zip(ci, cw)])
                if best is None or err < best[0]:
                    best = (err, Hc)
            if best is not None:
                Ha = best[1]
            H = Ha
            anchored += 1
        elif H is not None and prev_g is not None:
            A, ninl = track(prev_g, g)
            if A is None or ninl < 25:
                lost += 1            # keep the last H (hold position)
            else:
                H = H @ S_INV @ A @ S
                tracked += 1
        prev_g = g
        if H is None:
            continue
        h, w = frame.shape[:2]
        if MOSAIC and (fi // step) % 6 == 0:
            if "canvas" not in MOS:
                MOS["canvas"] = np.zeros((1000, 1000, 3), np.uint8)
            Mw = np.array([[25.0, 0, 375.0], [0, -25.0, 625.0], [0, 0, 1.0]])
            wp = cv2.warpPerspective(frame, Mw @ H, (1000, 1000))
            empty = MOS["canvas"].sum(axis=2) == 0
            MOS["canvas"][empty] = wp[empty]
        cx, cy = fx.pix_to_world(w / 2.0, h / 2.0, H)
        path.append((fi / fps, cx, cy, m_per_px(H, w / 2.0, h / 2.0)))
        for (bx, by, area) in denim_blobs(frame):
            s = m_per_px(H, bx, by)
            if AREA_MIN_M2 <= area * s * s <= AREA_MAX_M2:
                pts.append(fx.pix_to_world(bx, by, H))
    est = cluster(pts)
    if MOSAIC and "canvas" in MOS:
        cv = MOS.pop("canvas")
        for (x, y) in pts:
            cv2.circle(cv, (int(375 + 25 * x), int(625 - 25 * y)), 1, (0, 0, 255), -1)
        for i in range(1, len(path)):
            cv2.line(cv, (int(375 + 25 * path[i - 1][1]), int(625 - 25 * path[i - 1][2])),
                     (int(375 + 25 * path[i][1]), int(625 - 25 * path[i][2])), (0, 255, 255), 1)
        cv2.circle(cv, (375, 625), 6, (0, 255, 0), 2)
        import eval_harness as _eh
        _gt = _eh.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
        for v in _gt["victims"]:
            gx = v["marker_world"][0] - _gt["origin"][0]
            gy = v["marker_world"][1] - _gt["origin"][1]
            cv2.drawMarker(cv, (int(375 + 25 * gx), int(625 - 25 * gy)), (255, 0, 255),
                           cv2.MARKER_CROSS, 24, 2)
        cv2.imwrite(os.path.join(os.environ.get("TEMP", "."), f"mosaic_{world}.png"), cv)
    print(f"[{world}] frames anchored {anchored}, tracked {tracked}, lost {lost}; "
          f"{len(pts)} denim points; scale m/px first {path[0][3]:.4f} "
          f"last {path[-1][3]:.4f}; end pos ({path[-1][1]:.1f},{path[-1][2]:.1f})")
    if dump:
        for t, x, y, s in path[::40]:
            print(f"   t={t:5.1f} cam=({x:6.2f},{y:6.2f}) m/px={s:.4f}")
    return est


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    step = int(sys.argv[sys.argv.index("--step") + 1]) if "--step" in sys.argv else 4
    import eval_harness as eh
    for world in args or ["small", "medium", "large"]:
        est = run(world, step, "--dump" in sys.argv)
        gt = eh.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
        xy = np.array([[e[0], e[1]] for e in est]) if est else np.zeros((0, 2))
        for e in est:
            print(f"   est ({e[0]:+7.2f},{e[1]:+7.2f}) votes {e[2]}")
        rep, proxy = eh.victim_report(xy, gt["victims"], gt["origin"])
        print(rep)


if __name__ == "__main__":
    main()
