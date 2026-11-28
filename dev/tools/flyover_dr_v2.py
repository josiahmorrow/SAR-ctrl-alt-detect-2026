"""Victim localisation from the flyover by strap-down dead reckoning.

The flyover IMU is a body-frame sensor on a UAV that flies straight legs
and turns on the spot. Read as that it needs no fitted constants:

    forward speed  v(t)   = integral of acc_x        (body x = forward)
    yaw            psi(t) = integral of gyro_z
    position       p(t)   = p_ref + integral of v * heading(psi) dt

The origin mat fixes the ground frame, the metres-per-pixel scale and the
start position while it is in view. Two conventions are MEASURED from the
video rather than assumed: which image direction is "forward" (floor flow
on a straight leg) and the sign of yaw (image rotation during a turn on
the spot, where there is no parallax). One geometric check is applied to
the mat calibration: a camera looking straight down maps pixels (x right,
y down) to a right-handed ground frame by a reflection, so if the assumed
marker roles give a proper rotation the X and Y axes are swapped.

Victim evidence is a connected patch of denim blue with the visible area
of a person's legs, seen in several frames at one ground position.

A denim patch is then sized up as an object (_blue_object): a person seen
from above covers at most about 0.9 m2 whatever they wear, so a patch that
is part of a larger blue-green object (a blanket, a machine), or sits in a
field of other blue patches (striped bedding), is not counted, and the
ground such an object covers is remembered so that the few views in which
it merely looks small (cut by the frame edge, split by an occluder) do not
resurrect it. A location whose patch is narrower than a limb is a rim or
an edge, not a leg.

Known limits (see dev/docs/post_comp_log.md): flight height is taken as
constant after the mat leaves view; a person lying ON a large blue surface
would be discarded with it; the size and limb-width thresholds were set
with the three practice recordings in view.

Nothing here is keyed to a world, a recording name or a recording length.
"""
import csv
import os

import cv2
import numpy as np

DENIM_LO = np.array([95, 35, 25])
DENIM_HI = np.array([135, 255, 255])
AREA_MIN_M2 = 0.03          # visible denim of a lying person, lower bound
AREA_MAX_M2 = 0.80
FILL_MIN = 0.20             # a filled patch, not the outline of a frame
CLUSTER_R_M = 1.0           # the scoring radius
MIN_VOTES = 6
ELEVATED_LAMBDA = 0.17      # parallax of a surface about 0.65 m up at 4.5 m
ELEVATED_MIN_BASELINE_M = 0.6
FRAME_STEP = 4
CROP_M = 2.4                # neighbourhood examined round a denim patch
CROP_PX = 192               # 80 px per metre
# --- is the blue thing the size of a person? (see _blue_object) -----------
BLUE_LO = np.array([80, 25, 15])      # relaxed blue-green: the OBJECT the
BLUE_HI = np.array([140, 255, 255])   # denim patch is part of
OBJ_MAX_M2 = 1.00           # more than any person covers seen from above
                            # (a large adult, limbs apart: about 0.85 m2)
OTHER_BLUE_MAX_M2 = 0.60    # most of a second person's worth of denim nearby
BIG_OBJECT_RATIO = 0.5      # views that saw a too-big object here, per view
                            # that saw a person-sized one
THICK_MIN_M = 0.08          # narrower than a limb = a rim or an edge
FOOT_CELL_M = 0.10
FOOT_NEAR_CELLS = 4         # look 0.4 m round a view: the scatter of a cluster


def _rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def _load_imu(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append((float(row["timestamp"]), float(row["acc_x"]),
                             float(row["w_z"])))
            except (KeyError, ValueError):
                continue
    r = np.array(rows)
    t = r[:, 0]
    dt = np.diff(t, prepend=0.0)
    return t, dt, np.cumsum(r[:, 1] * dt), np.cumsum(r[:, 2] * dt), r[:, 2]


def _mat_similarity(fx, frame):
    """(camera-centre ground position, 2x2 pixel-offset -> ground, marker px)."""
    mk = fx.detect_aruco(frame)
    if 0 not in mk or len(mk) < 3:
        return None
    Ha = fx.build_calibration_H(mk)
    if Ha is None:
        return None
    ci = np.float64([c for m_ in mk.values() for c in m_])
    cw = np.float64([fx.pix_to_world(c[0], c[1], Ha) for c in ci])
    A = np.hstack([ci, np.ones((len(ci), 1))])
    sol, *_ = np.linalg.lstsq(A, cw, rcond=None)
    L, off = sol[:2].T, sol[2]
    if np.linalg.det(L) > 0:                 # handedness check (see header)
        P = np.array([[0.0, 1.0], [1.0, 0.0]])
        L, off = P @ L, P @ off
    U, S, Vt = np.linalg.svd(L)
    Ln = U @ Vt * S.mean()
    h, w = frame.shape[:2]
    side = np.mean([np.linalg.norm(mk[0][(i + 1) % 4] - mk[0][i]) for i in range(4)])
    return off + L @ np.array([w / 2.0, h / 2.0]), Ln, side


def _flow(cap, a, b):
    cap.set(cv2.CAP_PROP_POS_FRAMES, a)
    ok0, fa = cap.read()
    cap.set(cv2.CAP_PROP_POS_FRAMES, b)
    ok1, fb = cap.read()
    if not (ok0 and ok1):
        return None, None
    ga = cv2.cvtColor(cv2.resize(fa, None, fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(cv2.resize(fb, None, fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(ga, 600, 0.01, 10)
    if p0 is None:
        return None, None
    p1, st, _ = cv2.calcOpticalFlowPyrLK(ga, gb, p0, None, winSize=(21, 21),
                                         maxLevel=3)
    ok = st.ravel() == 1
    if ok.sum() < 20:
        return None, None
    return p0[ok].reshape(-1, 2) * 2.0, p1[ok].reshape(-1, 2) * 2.0


def _denim_patches(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, DENIM_LO, DENIM_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m, 8)
    out = []
    for i in range(1, n):
        area = float(stats[i, cv2.CC_STAT_AREA])
        if area < 150:
            continue
        if area / float(stats[i, cv2.CC_STAT_WIDTH] * stats[i, cv2.CC_STAT_HEIGHT]) < FILL_MIN:
            continue
        out.append((cents[i][0], cents[i][1], area, i))
    return out, hsv, lab


def _metric_crop(img, cx, cy, s_r, interp=cv2.INTER_AREA):
    """CROP_PX square crop covering CROP_M metres centred on a pixel; areas
    outside the frame are zero."""
    half = CROP_M / (2.0 * s_r)
    M = np.array([[CROP_PX / (2.0 * half), 0.0, CROP_PX / 2.0 - cx * CROP_PX / (2.0 * half)],
                  [0.0, CROP_PX / (2.0 * half), CROP_PX / 2.0 - cy * CROP_PX / (2.0 * half)]])
    return cv2.warpAffine(img, M, (CROP_PX, CROP_PX), flags=interp,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def _blue_object(bgr, comp, valid):
    """Size up what a denim patch belongs to, on a metric crop.

    A person seen from above is at most about 0.9 m2, whatever they wear. So:
    grow the patch through everything blue-green that touches it (shadowed
    denim, a blue top, the rest of a blanket, the rest of a machine) and
    measure it. Returns (object area m2, other denim area m2 in the
    neighbourhood, object mask). Pixels outside the video frame are unknown
    and count as nothing, so an object cut by the frame edge reads small."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ok = valid > 0
    c = (comp > 0) & ok
    m = ((cv2.inRange(hsv, BLUE_LO, BLUE_HI) > 0) | c) & ok
    _, lab = cv2.connectedComponents(m.astype(np.uint8), connectivity=8)
    ids = np.unique(lab[c])
    reg = np.isin(lab, ids[ids > 0])
    den = (cv2.inRange(hsv, DENIM_LO, DENIM_HI) > 0) & ok
    a = (CROP_M / CROP_PX) ** 2
    return float(reg.sum()) * a, float((den & ~reg).sum()) * a, reg


def locate_victims(video_path, imu_path, fx, log=print):
    """Return [(x, y, votes)] in the origin-mat frame, strongest first."""
    t, dt, v, psi, gz = _load_imu(imu_path)
    cap = cv2.VideoCapture(video_path)
    n = min(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), len(t))

    anchors, miss, fi = [], 0, 0
    while fi < n and miss < 80:
        ok, frame = cap.read()
        if not ok:
            break
        if fi % 2 == 0:
            a = _mat_similarity(fx, frame)
            if a is not None and a[2] >= 25:
                anchors.append((fi, a[0], a[1]))
                miss = 0
            elif anchors:
                miss += 1
        fi += 1
    if not anchors:
        raise RuntimeError("origin mat not found in the flyover")
    r, c_r, L_r = anchors[-1]
    s_r = float(np.sqrt(abs(np.linalg.det(L_r))))
    N_r = L_r / s_r

    sigma = 1.0
    turn = [i for i in range(r, n - 12) if abs(gz[i]) > 0.3 and abs(v[i]) < 0.05
            and abs(gz[i + 10]) > 0.3]
    if turn:
        a = turn[len(turn) // 8]
        q0, q1 = _flow(cap, a, a + 10)
        if q0 is not None:
            A, _ = cv2.estimateAffinePartial2D(q1, q0, method=cv2.RANSAC,
                                               ransacReprojThreshold=2.0)
            if A is not None:
                meas = np.arctan2(A[1, 0], A[0, 0])
                dpsi = psi[a + 10] - psi[a]
                best = None
                for sg in (1.0, -1.0):
                    Pm = np.linalg.inv(N_r) @ _rot(sg * dpsi) @ N_r
                    e = abs(np.angle(np.exp(1j * (np.arctan2(Pm[1, 0], Pm[0, 0]) - meas))))
                    if best is None or e < best[0]:
                        best = (e, sg)
                sigma = best[1]
    u = np.array([0.0, -1.0])
    straight = [i for i in range(r, n - 8) if v[i] > 1.0 and abs(gz[i]) < 0.01
                and v[i + 6] > 1.0 and abs(gz[i + 6]) < 0.01]
    if straight:
        q0, q1 = _flow(cap, straight[0], straight[0] + 6)
        if q0 is not None:
            d = np.median(q0 - q1, axis=0)
            if np.linalg.norm(d) > 1.0:
                ax = int(np.argmax(np.abs(d)))
                u = (np.array([np.sign(d[0]), 0.0]) if ax == 0
                     else np.array([0.0, np.sign(d[1])]))

    def O(i):
        return _rot(sigma * (psi[i] - psi[r])) @ N_r

    p = np.zeros((n, 2))
    p[r] = c_r
    for i in range(r + 1, n):
        p[i] = p[i - 1] + v[i] * dt[i] * (O(i) @ u)
    for i in range(r - 1, -1, -1):
        p[i] = p[i + 1] - v[i + 1] * dt[i + 1] * (O(i + 1) @ u)
    drift = max(float(np.linalg.norm(p[a[0]] - a[1])) for a in anchors)
    log(f"[flyover_dr] mat seen in frames {anchors[0][0]}-{r}, {s_r:.4f} m/px, "
        f"dead reckoning vs mat {drift:.2f} m, yaw sign {sigma:+.0f}, forward {u}")

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    obs = []                # (x, y, cam_x, cam_y, patch thickness m)
    cover = {}              # ground cell -> views in which a too-big blue object lay on it
    n_big = n_ctx = 0
    half_px = CROP_PX / 2.0
    m_per_cpx = CROP_M / CROP_PX
    for i in range(n):
        if not cap.grab():
            break
        if i % FRAME_STEP or i < anchors[0][0]:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        h, w = frame.shape[:2]
        ctr = np.array([w / 2.0, h / 2.0])
        Oi = O(i)
        patches, hsv, lab = _denim_patches(frame)
        valid_full = None
        for (bx, by, area, li) in patches:
            if not (AREA_MIN_M2 <= area * s_r * s_r <= AREA_MAX_M2):
                continue
            g = p[i] + s_r * Oi @ (np.array([bx, by]) - ctr)
            comp = (lab == li).astype(np.uint8) * 255
            if valid_full is None:
                valid_full = np.full((h, w), 255, np.uint8)
            obj, other, reg = _blue_object(
                _metric_crop(frame, bx, by, s_r),
                _metric_crop(comp, bx, by, s_r, cv2.INTER_NEAREST),
                _metric_crop(valid_full, bx, by, s_r, cv2.INTER_NEAREST))
            if obj > OBJ_MAX_M2:
                # Bigger than a person: remember the ground it covers.
                n_big += 1
                vs, us = np.nonzero(reg[::3, ::3])
                off = np.column_stack([(us * 3 - half_px) * m_per_cpx,
                                       (vs * 3 - half_px) * m_per_cpx])
                cells = np.floor((g + off @ Oi.T) / FOOT_CELL_M).astype(int)
                for cl in set(map(tuple, cells)):
                    cover[cl] = cover.get(cl, 0) + 1
                continue
            if other > OTHER_BLUE_MAX_M2:
                n_ctx += 1          # one blue patch among many: a pattern
                continue
            thick = 2.0 * float(cv2.distanceTransform(comp, cv2.DIST_L2, 3).max()) * s_r
            obs.append((g[0], g[1], p[i][0], p[i][1], thick))
    cap.release()
    log(f"[flyover_dr] denim views: {len(obs)} person-sized, {n_big} of an object "
        f"over {OBJ_MAX_M2:.1f} m2, {n_ctx} in a field of blue")

    out, rest = [], list(obs)
    while rest:
        best = max(rest, key=lambda a: sum(
            1 for b in rest if np.hypot(a[0] - b[0], a[1] - b[1]) < CLUSTER_R_M))
        mem = [b for b in rest if np.hypot(best[0] - b[0], best[1] - b[1]) < CLUSTER_R_M]
        if len(mem) < MIN_VOTES:
            break
        M = np.array(mem)
        dP, dC = M[:, :2] - M[:, :2].mean(0), M[:, 2:4] - M[:, 2:4].mean(0)
        base = float(np.sqrt((dC * dC).sum(1).mean()))
        lam = -float((dP * dC).sum() / max((dC * dC).sum(), 1e-9))
        x, y = float(np.median(M[:, 0])), float(np.median(M[:, 1]))
        big = float(np.median([
            max(cover.get((int(np.floor(b[0] / FOOT_CELL_M)) + da,
                           int(np.floor(b[1] / FOOT_CELL_M)) + db), 0)
                for da in range(-FOOT_NEAR_CELLS, FOOT_NEAR_CELLS + 1)
                for db in range(-FOOT_NEAR_CELLS, FOOT_NEAR_CELLS + 1))
            for b in mem]))
        thick = float(np.median(M[:, 4]))
        if big > BIG_OBJECT_RATIO * len(mem):
            # Left-over views of something that most views show to be too big
            # (cut by the frame edge, or split by an occluder in these ones).
            log(f"[flyover_dr] dropped ({x:.2f},{y:.2f}): {len(mem)} person-sized "
                f"views against {big:.0f} of a larger blue object there")
        elif thick < THICK_MIN_M:
            log(f"[flyover_dr] dropped ({x:.2f},{y:.2f}): patch {thick:.3f} m "
                f"across, narrower than a limb")
        elif base >= ELEVATED_MIN_BASELINE_M and lam > ELEVATED_LAMBDA:
            # Something well above the floor slides against the camera's motion
            # (parallax). Victims lie on the floor; a seat or shelf does not.
            log(f"[flyover_dr] dropped elevated object at ({x:.2f},{y:.2f}) "
                f"(parallax {lam:.2f} over {base:.2f} m)")
        else:
            log(f"[flyover_dr] kept ({x:.2f},{y:.2f}) n={len(mem)} big={big:.0f} "
                f"thick={thick:.3f} lam={lam:.2f} base={base:.2f}")
            out.append((x, y, len(mem)))
        rest = [b for b in rest
                if np.hypot(best[0] - b[0], best[1] - b[1]) >= CLUSTER_R_M * 1.5]
    return out
