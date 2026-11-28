"""Prototype: victim localisation from the flyover by strap-down dead reckoning.

The flyover IMU is a body-frame sensor on a UAV that flies straight legs and
turns on the spot. Read as that, it needs no fitted constants:

  forward speed  v(t)   = integral of acc_x              (body x = forward)
  yaw            psi(t) = integral of gyro_z
  height         z(t)   = double integral of (acc_z - g)
  position       p(t)   = p_ref + integral of v * heading(psi) dt

The origin marker fixes everything else while it is in view: the ground
frame, the metres-per-pixel scale and its dependence on height. Two
conventions are MEASURED from the video rather than assumed: which image
direction is "forward" (floor flow on a straight leg) and the sign of yaw
(image rotation during a turn on the spot, where there is no parallax).

    python dev/tools/flyover_dr.py small medium large [--mosaic] [--dump]

Offline evaluation against the world file (eval_harness) is for development
only; nothing here is used by the controller yet.
"""
import csv
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
G = 9.81
DENIM_LO, DENIM_HI = np.array([95, 35, 25]), np.array([135, 255, 255])
AREA_MIN_M2, AREA_MAX_M2 = 0.03, 0.80
CLUSTER_R_M, MIN_VOTES = 1.0, 6
MOSAIC = "--mosaic" in sys.argv
STATS = {}
DUMP = "--dump" in sys.argv


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def load_imu(path):
    r = np.array([[float(v) for v in row.values()]
                  for row in csv.DictReader(open(path))])
    t = r[:, 0]
    dt = np.diff(t, prepend=0.0)
    v = np.cumsum(r[:, 1] * dt)                      # forward speed
    psi = np.cumsum(r[:, 6] * dt)                    # yaw
    # Start and end in level flight or at rest: net vertical velocity change
    # is zero, so the sensor's own mean is the gravity it measures.
    vz = np.cumsum((r[:, 3] - np.average(r[:, 3], weights=np.maximum(dt, 1e-9))) * dt)
    z = np.cumsum(vz * dt)                           # height change
    return {"t": t, "dt": dt, "v": v, "psi": psi, "z": z, "gz": r[:, 6]}


def marker_similarity(frame):
    """(centre_world, L 2x2 pixel-offset->world) from the origin mat, or None."""
    mk = fx.detect_aruco(frame)
    if 0 not in mk or len(mk) < 3:
        return None
    Ha = fx.build_calibration_H(mk)
    if Ha is None:
        return None
    ci = np.float64([c for m_ in mk.values() for c in m_])
    cw = np.float64([fx.pix_to_world(c[0], c[1], Ha) for c in ci])
    # least-squares affine pixel->world, then the nearest scaled orthogonal map
    A = np.hstack([ci, np.ones((len(ci), 1))])
    sol, *_ = np.linalg.lstsq(A, cw, rcond=None)     # 3x2
    L = sol[:2].T
    off = sol[2]
    # Handedness check. The frame is right-handed with Z up, and the camera
    # looks straight down, so pixel (x right, y down) -> ground (X, Y) must be
    # a reflection (det < 0). If the marker roles assumed by the calibration
    # give det > 0 the X and Y markers are the other way round: swap axes.
    if np.linalg.det(L) > 0:
        P = np.array([[0.0, 1.0], [1.0, 0.0]])
        L, off = P @ L, P @ off
    sol = np.vstack([L.T, off])
    U, S, Vt = np.linalg.svd(L)
    Ln = U @ Vt * S.mean()
    h, w = frame.shape[:2]
    ctr = np.array([w / 2.0, h / 2.0])
    c_world = sol[2] + L @ ctr
    side = np.mean([np.linalg.norm(mk[0][(i + 1) % 4] - mk[0][i]) for i in range(4)])
    return c_world, Ln, side


def flow(cap, a, b):
    """Tracked points in frame a and frame b (half resolution)."""
    cap.set(cv2.CAP_PROP_POS_FRAMES, a)
    _, fa = cap.read()
    cap.set(cv2.CAP_PROP_POS_FRAMES, b)
    _, fb = cap.read()
    ga = cv2.cvtColor(cv2.resize(fa, None, fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(cv2.resize(fb, None, fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(ga, 600, 0.01, 10)
    if p0 is None:
        return None, None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(ga, gb, p0, None, winSize=(21, 21), maxLevel=3)
    ok = st.ravel() == 1
    return p0[ok].reshape(-1, 2) * 2.0, p1[ok].reshape(-1, 2) * 2.0


def denim_blobs(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, DENIM_LO, DENIM_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 150:
            continue
        x, y, w_, h_ = stats[i, :4]
        sub = lab[y:y + h_, x:x + w_] == i
        px = hsv[y:y + h_, x:x + w_][sub]
        STATS[(round(float(cents[i][0]), 1), round(float(cents[i][1]), 1))] = (
            float(np.median(px[:, 0])), float(np.median(px[:, 1])), float(np.median(px[:, 2])),
            float(stats[i, cv2.CC_STAT_AREA]) / float(w_ * h_), max(w_, h_) / max(1.0, min(w_, h_)))
        out.append((cents[i][0], cents[i][1], stats[i, cv2.CC_STAT_AREA]))
    return out


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
        if len(mem[0]) > 2:
            print(f'   cluster ({out[-1][0]:+.2f},{out[-1][1]:+.2f}) votes {len(mem)} t {min(q[2] for q in mem):.1f}-{max(q[2] for q in mem):.1f}')
        pts = [q for q in pts
               if np.hypot(best[0] - q[0], best[1] - q[1]) >= CLUSTER_R_M * 1.5]
    return out


def run(world):
    vid = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
    imu = load_imu(os.path.join(REPO, "recordings", f"{world}_world_flyover.csv"))
    cap = cv2.VideoCapture(vid)
    n = min(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), len(imu["t"]))

    # 1. anchors from the origin mat
    anchors, miss, fi = [], 0, 0
    while fi < n and miss < 80:
        ok, frame = cap.read()
        if not ok:
            break
        if fi % 2 == 0:
            a = marker_similarity(frame)
            if a is not None and a[2] >= 25:
                anchors.append((fi, a[0], a[1], np.sqrt(abs(np.linalg.det(a[1])))))
                miss = 0
            elif anchors:
                miss += 1
        fi += 1
    r, c_r, L_r, s_r = anchors[-1]
    N_r = L_r / s_r

    # 2. scale against height: z_imu = k * s + c, from the climb in view
    zs = np.array([imu["z"][a[0]] for a in anchors])
    ss = np.array([a[3] for a in anchors])
    if np.ptp(zs) > 0.3 and '--flat' not in sys.argv:
        k, c0 = np.polyfit(ss, zs, 1)
    else:                                   # no climb seen: hold the scale
        k, c0 = None, None
    scale = (lambda i: (imu["z"][i] - c0) / k) if k else (lambda i: s_r)

    # 3. conventions measured from the video
    v, psi, gz, dt = imu["v"], imu["psi"], imu["gz"], imu["dt"]
    sigma, u = 1.0, None
    turn = [i for i in range(r, n - 12) if abs(gz[i]) > 0.3 and abs(v[i]) < 0.05
            and abs(gz[i + 10]) > 0.3]
    if turn:
        a = turn[len(turn) // 8]
        q0, q1 = flow(cap, a, a + 10)
        if q0 is not None and len(q0) >= 20:
            A, _ = cv2.estimateAffinePartial2D(q1, q0, method=cv2.RANSAC,
                                               ransacReprojThreshold=2.0)
            meas = np.arctan2(A[1, 0], A[0, 0])          # cur -> prev rotation
            dpsi = psi[a + 10] - psi[a]
            pred = {}
            for sg in (1.0, -1.0):
                P = np.linalg.inv(N_r) @ rot(sg * dpsi) @ N_r
                pred[sg] = np.arctan2(P[1, 0], P[0, 0])
            sigma = min(pred, key=lambda sg: abs(np.angle(np.exp(1j * (pred[sg] - meas)))))
            print(f"   yaw sign {sigma:+.0f} (image {np.degrees(meas):.1f} deg, "
                  f"gyro {np.degrees(dpsi):.1f} deg)")
    straight = [i for i in range(r, n - 8) if v[i] > 1.0 and abs(gz[i]) < 0.01
                and v[i + 6] > 1.0 and abs(gz[i + 6]) < 0.01]
    if straight:
        a = straight[0]
        q0, q1 = flow(cap, a, a + 6)
        d = np.median(q0 - q1, axis=0)                   # prev - cur
        u = d / np.linalg.norm(d)
        # snap to the nearest image axis: the camera is mounted square
        ax = np.argmax(np.abs(u))
        u = np.array([np.sign(u[0]), 0.0]) if ax == 0 else np.array([0.0, np.sign(u[1])])
        print(f"   forward in image = {u}")
    else:
        u = np.array([0.0, -1.0])

    def O(i):
        return rot(sigma * (psi[i] - psi[r])) @ N_r

    # 4. dead-reckoned camera centre, both ways from the reference frame
    p = np.zeros((n, 2))
    p[r] = c_r
    for i in range(r + 1, n):
        p[i] = p[i - 1] + v[i] * dt[i] * (O(i) @ u)
    for i in range(r - 1, -1, -1):
        p[i] = p[i + 1] - v[i + 1] * dt[i + 1] * (O(i + 1) @ u)
    err = [np.linalg.norm(p[a[0]] - a[1]) for a in anchors]
    print(f"[{world}] anchors {len(anchors)} (frames {anchors[0][0]}-{r}), "
          f"m/px at ref {s_r:.4f}, dead-reckoning vs marker over the anchored "
          f"span: max {max(err):.2f} m; scale-height fit "
          f"{'k=%.0f px' % k if k else 'none'}; m/px range "
          f"{min(scale(i) for i in range(r, n, 50)):.4f}-"
          f"{max(scale(i) for i in range(r, n, 50)):.4f}")

    # 5. detections projected to the floor
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    pts, canvas = [], np.zeros((1000, 1000, 3), np.uint8)
    for i in range(n):
        ok = cap.grab()
        if not ok:
            break
        if i % 4 or i < anchors[0][0]:
            continue
        ok, frame = cap.retrieve()
        h, w = frame.shape[:2]
        ctr = np.array([w / 2.0, h / 2.0])
        s, Oi = scale(i), O(i)
        if MOSAIC and (i // 4) % 6 == 0:
            Hm = np.eye(3)
            Hm[:2, :2] = s * Oi
            Hm[:2, 2] = p[i] - s * Oi @ ctr
            Mw = np.array([[25.0, 0, 375.0], [0, -25.0, 625.0], [0, 0, 1.0]])
            wp = cv2.warpPerspective(frame, Mw @ Hm, (1000, 1000))
            empty = canvas.sum(axis=2) == 0
            canvas[empty] = wp[empty]
        for (bx, by, area) in denim_blobs(frame):
            if AREA_MIN_M2 <= area * s * s <= AREA_MAX_M2:
                pts.append(tuple(p[i] + s * Oi @ (np.array([bx, by]) - ctr)) + (imu['t'][i], bx, by, i, area * s * s) + STATS.get((round(float(bx), 1), round(float(by), 1)), (0, 0, 0, 0, 0)))
    est = cluster(pts)
    for (ex, ey, nv) in est:
        mem = [q for q in pts if np.hypot(q[0] - ex, q[1] - ey) < CLUSTER_R_M]
        P = np.array([[q[0], q[1]] for q in mem])
        C = np.array([p[q[5]] for q in mem])
        dP, dC = P - P.mean(0), C - C.mean(0)
        lam = -float((dP * dC).sum() / max((dC * dC).sum(), 1e-9))
        print(f"   height test ({ex:+.2f},{ey:+.2f}) n={nv} cam spread "
              f"{np.sqrt((dC * dC).sum(1).mean()):.2f} m  lambda {lam:+.3f}  "
              f"resid {np.sqrt(((dP + lam * dC) ** 2).sum(1).mean()):.2f} m")
        print('      H %.0f S %.0f V %.0f fill %.2f aspect %.1f area %.2f' % tuple(np.median([q[k] for q in mem]) for k in (7, 8, 9, 10, 11, 6)))
    if '--sheet' in sys.argv:
        tiles = []
        for (ex, ey, nv) in est:
            mem = sorted([q for q in pts if np.hypot(q[0] - ex, q[1] - ey) < CLUSTER_R_M], key=lambda q: q[5])
            q = mem[len(mem) // 2]
            cap.set(cv2.CAP_PROP_POS_FRAMES, q[5])
            _, fr = cap.read()
            x0, y0 = int(max(0, min(fr.shape[1] - 480, q[3] - 240))), int(max(0, min(fr.shape[0] - 480, q[4] - 240)))
            crop = fr[y0:y0 + 480, x0:x0 + 480].copy()
            cv2.circle(crop, (int(q[3] - x0), int(q[4] - y0)), 6, (0, 0, 255), 2)
            cv2.putText(crop, f'({ex:.1f},{ey:.1f}) n={nv} a={np.median([m_[6] for m_ in mem]):.2f}', (5, 22), 0, 0.6, (0, 255, 255), 2)
            tiles.append(cv2.resize(crop, (320, 320)))
        while len(tiles) % 5:
            tiles.append(np.zeros((320, 320, 3), np.uint8))
        cv2.imwrite(os.path.join(os.environ.get('TEMP', '.'), f'dr_sheet_{world}.png'), np.vstack([np.hstack(tiles[k:k + 5]) for k in range(0, len(tiles), 5)]))
    if DUMP:
        for i in range(0, n, 300):
            print(f"   t={imu['t'][i]:5.1f} cam=({p[i][0]:6.2f},{p[i][1]:6.2f}) "
                  f"m/px={scale(i):.4f} yaw={np.degrees(psi[i]):7.1f}")
    return est, pts, p, canvas


def main():
    import eval_harness as eh
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    for world in args or ["small", "medium", "large"]:
        est, pts, p, canvas = run(world)
        gt = eh.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
        xy = np.array([[e[0], e[1]] for e in est]) if est else np.zeros((0, 2))
        rep, _ = eh.victim_report(xy, gt["victims"], gt["origin"])
        print(rep)
        if MOSAIC:
            for (x, y, *_r) in pts:
                cv2.circle(canvas, (int(375 + 25 * x), int(625 - 25 * y)), 1, (0, 0, 255), -1)
            for i in range(4, len(p), 4):
                cv2.line(canvas, (int(375 + 25 * p[i - 4][0]), int(625 - 25 * p[i - 4][1])),
                         (int(375 + 25 * p[i][0]), int(625 - 25 * p[i][1])), (0, 255, 255), 1)
            for v_ in gt["victims"]:
                gx = v_["marker_world"][0] - gt["origin"][0]
                gy = v_["marker_world"][1] - gt["origin"][1]
                cv2.drawMarker(canvas, (int(375 + 25 * gx), int(625 - 25 * gy)),
                               (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.imwrite(os.path.join(os.environ.get("TEMP", "."),
                                     f"dr_mosaic_{world}.png"), canvas)


if __name__ == "__main__":
    main()





