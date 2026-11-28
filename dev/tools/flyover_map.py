"""Offline prototype: wall map and obstacle layer from the flyover.

Camera poses come from the same strap-down dead reckoning as flyover_dr.py
(forward speed = integral of acc_x, yaw = integral of gyro_z, frame / scale /
start from the origin mat, yaw sign and forward direction measured from the
video). On top of that, HEIGHT comes from parallax:

  On a straight leg the camera moves a known distance d between two frames.
  A floor point then moves d / s pixels (s = metres per pixel on the floor).
  A point at height h moves r times as far, r = H / (H - h), H = camera
  height. So r, measured per tracked feature, gives the relative height
  h / H = 1 - 1 / r with no knowledge of H, and the feature's true ground
  position is p_cam + (s / r) * O * (pixel - centre): the parallax shift of
  tall things is removed instead of avoided.

Features are binned on a 0.05 m ground grid by relative height:
  wall-height  (h/H >= WALL_REL)        -> wall map (graded format)
  obstacle     (OBST_REL <= h/H < WALL) -> obstacle layer
  floor        (h/H < OBST_REL)         -> seen-free evidence

    python dev/tools/flyover_map.py small medium large [--out DIR]

Writes <out>/<world>_walls.png (600x600, 0 = wall, 255 = free, centred on
the detected walls' bounding box like the supervisor's ground truth),
<world>_obst.png, <world>_debug.png, <world>_mosaic.png, and prints offline
scores against the world file (eval_harness; development only).
Nothing here is keyed to a world or a recording, and nothing is used by the
controller yet.
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
import flyover_dr as dr                  # noqa: E402  (pose helpers, read only)

REPO = os.path.dirname(os.path.dirname(SOL))
RES = 0.05                 # m per cell (graded map resolution)
GRID = 1200                # working grid: 60 m square centred on the origin
PAIR_GAP = 6               # frames between the two views of a pair
PAIR_EVERY = 3             # use every n-th frame as the second view
MIN_BASE_M = 0.08          # minimum camera travel within a pair
WALL_REL = 0.36            # h/H at or above this = wall-height
OBST_REL = 0.07            # h/H at or above this = above the floor
MAX_REL = 0.75             # above this the match is not credible
PERP_TOL = 0.12            # sideways flow / along-track flow
FB_TOL_PX = 1.0            # forward-backward tracking error
SWEEP = "--sweep" in sys.argv
WALL_MIN_VOTES = 6 if SWEEP else 3
OBST_MIN_VOTES = 6 if SWEEP else 3
PAIR_EVERY = 3 if SWEEP else 2
SWEEP_REL = np.concatenate([np.arange(0.0, 0.20, 0.02),
                            np.arange(0.20, 0.70, 0.04)]).astype(np.float32)  # h/H hypotheses
SWEEP_MIN_FLOOR_PX = 12.0  # floor shift needed to resolve heights
SWEEP_TEX_MIN = 1.5        # mean gradient along the motion, grey levels
SWEEP_COST_MAX = 6.0       # mean absolute difference at the best height


def poses(video_path, imu_path, log=print):
    """Dead-reckoned camera centre p[i] and orientation O(i) per frame."""
    t, dt, v, psi, gz = dr._load_imu(imu_path)
    cap = cv2.VideoCapture(video_path)
    n = min(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), len(t))
    anchors, miss, fi = [], 0, 0
    while fi < n and miss < 80:
        ok, frame = cap.read()
        if not ok:
            break
        if fi % 2 == 0:
            a = dr._mat_similarity(fx, frame)
            if a is not None and a[2] >= 25:
                anchors.append((fi, a[0], a[1]))
                miss = 0
            elif anchors:
                miss += 1
        fi += 1
    if not anchors:
        raise RuntimeError("origin mat not found")
    r, c_r, L_r = anchors[-1]
    s_r = float(np.sqrt(abs(np.linalg.det(L_r))))
    N_r = L_r / s_r

    sigma = 1.0
    turn = [i for i in range(r, n - 12) if abs(gz[i]) > 0.3 and abs(v[i]) < 0.05
            and abs(gz[i + 10]) > 0.3]
    if turn:
        a = turn[len(turn) // 8]
        q0, q1 = dr._flow(cap, a, a + 10)
        if q0 is not None:
            A, _ = cv2.estimateAffinePartial2D(q1, q0, method=cv2.RANSAC,
                                               ransacReprojThreshold=2.0)
            if A is not None:
                meas = np.arctan2(A[1, 0], A[0, 0])
                dpsi = psi[a + 10] - psi[a]
                best = None
                for sg in (1.0, -1.0):
                    Pm = np.linalg.inv(N_r) @ dr._rot(sg * dpsi) @ N_r
                    e = abs(np.angle(np.exp(1j * (np.arctan2(Pm[1, 0], Pm[0, 0]) - meas))))
                    if best is None or e < best[0]:
                        best = (e, sg)
                sigma = best[1]
    u = np.array([0.0, -1.0])
    straight = [i for i in range(r, n - 8) if v[i] > 1.0 and abs(gz[i]) < 0.01
                and v[i + 6] > 1.0 and abs(gz[i + 6]) < 0.01]
    if straight:
        q0, q1 = dr._flow(cap, straight[0], straight[0] + 6)
        if q0 is not None:
            d = np.median(q0 - q1, axis=0)
            if np.linalg.norm(d) > 1.0:
                ax = int(np.argmax(np.abs(d)))
                u = (np.array([np.sign(d[0]), 0.0]) if ax == 0
                     else np.array([0.0, np.sign(d[1])]))
    cap.release()

    def O(i):
        return dr._rot(sigma * (psi[i] - psi[r])) @ N_r

    p = np.zeros((n, 2))
    p[r] = c_r
    for i in range(r + 1, n):
        p[i] = p[i - 1] + v[i] * dt[i] * (O(i) @ u)
    for i in range(r - 1, -1, -1):
        p[i] = p[i + 1] - v[i + 1] * dt[i + 1] * (O(i + 1) @ u)
    log(f"[flyover_map] mat frames {anchors[0][0]}-{r}, {s_r:.4f} m/px, "
        f"yaw sign {sigma:+.0f}, forward {u}")
    return {"n": n, "p": p, "O": O, "s": s_r, "u": u, "v": v, "dt": dt,
            "gz": gz, "r": r, "first": anchors[0][0]}


def w2g(xy):
    """Origin-frame metres -> working-grid (col, row)."""
    return (np.round(xy[:, 0] / RES + GRID / 2).astype(int),
            np.round(GRID / 2 - xy[:, 1] / RES).astype(int))


def build(world, out_dir, log=print):
    vid = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
    imu = os.path.join(REPO, "recordings", f"{world}_world_flyover.csv")
    P = poses(vid, imu, log)
    n, p, O, s, u, v, dt, gz = (P[k] for k in ("n", "p", "O", "s", "u", "v", "dt", "gz"))
    wall = np.zeros((GRID, GRID), np.uint16)
    obst = np.zeros((GRID, GRID), np.uint16)
    floor = np.zeros((GRID, GRID), np.uint16)
    mosaic = np.zeros((GRID // 2, GRID // 2, 3), np.uint8)   # 0.10 m per px
    cap = cv2.VideoCapture(vid)
    ring = {}
    used = npts = 0
    lk = dict(winSize=(21, 21), maxLevel=4)
    for i in range(n):
        if not cap.grab():
            break
        if i < P["r"]:
            continue
        need = (i % PAIR_EVERY == 0) or ((i + PAIR_GAP) % PAIR_EVERY == 0)
        if not need:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        h, w = frame.shape[:2]
        ctr = np.array([w / 2.0, h / 2.0])
        g = cv2.cvtColor(cv2.resize(frame, None, fx=0.5, fy=0.5,
                                    interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        ring[i] = g
        for k in [k for k in ring if k < i - PAIR_GAP]:
            del ring[k]
        # near-nadir colour mosaic (central strip only: little parallax)
        if i % 12 == 0:
            Oi = O(i)
            Hm = np.eye(3)
            Hm[:2, :2] = s * Oi
            Hm[:2, 2] = p[i] - s * Oi @ ctr
            Mw = np.array([[10.0, 0, GRID / 4], [0, -10.0, GRID / 4], [0, 0, 1.0]])
            msk = np.zeros((h, w), np.uint8)
            cv2.circle(msk, (w // 2, h // 2), int(0.22 * w), 255, -1)
            wp = cv2.warpPerspective(frame, Mw @ Hm, mosaic.shape[1::-1])
            wm = cv2.warpPerspective(msk, Mw @ Hm, mosaic.shape[1::-1])
            sel = wm > 0
            mosaic[sel] = wp[sel]
        j = i - PAIR_GAP
        if i % PAIR_EVERY or j not in ring:
            continue
        # straight leg only: no rotation between the two views
        if np.abs(gz[j:i + 1]).max() > 0.02:
            continue
        d = float(np.sum(v[j + 1:i + 1] * dt[j + 1:i + 1]))
        if d < MIN_BASE_M:
            continue
        if not SWEEP:
            # Sparse, forward-backward-checked feature tracks (default).
            q0 = cv2.goodFeaturesToTrack(ring[j], 1500, 0.004, 5)
            if q0 is None:
                continue
            q1, st, _ = cv2.calcOpticalFlowPyrLK(ring[j], g, q0, None, **lk)
            qb, st2, _ = cv2.calcOpticalFlowPyrLK(g, ring[j], q1, None, **lk)
            ok = (st.ravel() == 1) & (st2.ravel() == 1) & \
                 (np.linalg.norm((qb - q0).reshape(-1, 2), axis=1) < FB_TOL_PX)
            a0 = q0.reshape(-1, 2)[ok] * 2.0
            a1 = q1.reshape(-1, 2)[ok] * 2.0
            if len(a0) < 30:
                continue
            fl = a0 - a1                      # prev - cur = u * (d / s) * r
            along = fl @ u
            perp = np.abs(fl @ np.array([-u[1], u[0]]))
            good = (along > 0) & (perp < PERP_TOL * np.maximum(along, 1.0) + 1.0)
            rr = along[good] * s / d
            pts = a1[good]
            keep = (rr > 0.85) & (rr < 1.0 / (1.0 - MAX_REL))
            rr, pts = rr[keep], pts[keep]
            # floor = the lowest well-populated height in view (see below)
            lowr = rr[rr < 1.2]
            if len(lowr) < 15:
                continue
            hist, edges = np.histogram(lowr, bins=14, range=(0.85, 1.2))
            r_floor = 0.5 * (edges[hist.argmax()] + edges[hist.argmax() + 1])
            rr = np.maximum(rr / r_floor, 1.0)
            rel = 1.0 - 1.0 / rr
            Oi = O(i)
            gxy = p[i] + (s / rr)[:, None] * ((pts - ctr) @ Oi.T)
            cx, cy = w2g(gxy)
            inb = (cx >= 0) & (cx < GRID) & (cy >= 0) & (cy < GRID)
            cx, cy, rel = cx[inb], cy[inb], rel[inb]
            np.add.at(wall, (cy[rel >= WALL_REL], cx[rel >= WALL_REL]), 1)
            mo = (rel >= OBST_REL) & (rel < WALL_REL)
            np.add.at(obst, (cy[mo], cx[mo]), 1)
            np.add.at(floor, (cy[rel < OBST_REL], cx[rel < OBST_REL]), 1)
            used += 1
            npts += len(rel)
            continue
        # --sweep: dense plane sweep. EXPERIMENTAL, does not work yet (labels
        # most of the floor as raised); kept for reference only.
        g0, g1 = ring[j].astype(np.float32), g.astype(np.float32)
        f0 = (d / s) * 0.5                    # floor shift, half-res pixels
        if f0 < SWEEP_MIN_FLOOR_PX:
            continue
        # Plane sweep along the known motion direction: for each height
        # hypothesis, shift the earlier frame by what that height predicts
        # and see how well it matches. Works on weak texture (wall tops),
        # where corner tracking finds nothing.
        gh, gw = g1.shape
        costs = np.empty((len(SWEEP_REL), gh, gw), np.float32)
        for k, relk in enumerate(SWEEP_REL):
            sh = f0 / (1.0 - relk)
            M = np.float32([[1, 0, -u[0] * sh], [0, 1, -u[1] * sh]])
            w0 = cv2.warpAffine(g0, M, (gw, gh), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=-1000)
            costs[k] = cv2.boxFilter(np.abs(g1 - w0), -1, (9, 9))
        kb = np.argmin(costs, axis=0)
        cb = np.take_along_axis(costs, kb[None], 0)[0]
        cmed = np.median(costs, axis=0)
        gu = cv2.Sobel(g1, cv2.CV_32F, int(abs(u[0])), int(abs(u[1])), ksize=3)
        tex = cv2.boxFilter(np.abs(gu), -1, (9, 9))
        okm = (tex > SWEEP_TEX_MIN) & (cb < SWEEP_COST_MAX) & (cb < 0.6 * cmed)
        mrg_x, mrg_y = int(0.1 * gw), int(0.1 * gh)
        okm[:mrg_y] = okm[-mrg_y:] = False
        okm[:, :mrg_x] = okm[:, -mrg_x:] = False
        okm[::2] = False
        okm[:, ::2] = False
        ys_, xs_ = np.nonzero(okm)
        if not len(xs_):
            continue
        kk = kb[ys_, xs_]
        # The floor is the lowest well-populated height in view. Measure it
        # per pair instead of trusting speed x time to the last few percent:
        # heights are then relative to the floor actually seen.
        low = kk[SWEEP_REL[kk] <= 0.16]
        if len(low) < 200:
            continue
        k0 = int(np.bincount(low).argmax())
        rr = (1.0 - SWEEP_REL[k0]) / (1.0 - SWEEP_REL[kk])
        keep = rr >= 0.97
        rr, xs_, ys_ = np.maximum(rr[keep], 1.0), xs_[keep], ys_[keep]
        rel = 1.0 - 1.0 / rr
        pts = np.stack([xs_, ys_], 1).astype(np.float64) * 2.0
        Oi = O(i)
        gxy = p[i] + (s / rr)[:, None] * ((pts - ctr) @ Oi.T)
        cx, cy = w2g(gxy)
        inb = (cx >= 0) & (cx < GRID) & (cy >= 0) & (cy < GRID)
        cx, cy, rel = cx[inb], cy[inb], rel[inb]
        np.add.at(wall, (cy[rel >= WALL_REL], cx[rel >= WALL_REL]), 1)
        mo = (rel >= OBST_REL) & (rel < WALL_REL)
        np.add.at(obst, (cy[mo], cx[mo]), 1)
        np.add.at(floor, (cy[rel < OBST_REL], cx[rel < OBST_REL]), 1)
        used += 1
        npts += len(rel)
    cap.release()
    log(f"[flyover_map] {world}: {used} frame pairs, {npts} height points; cells "
        f"wall {int((wall > 0).sum())} obstacle {int((obst > 0).sum())} "
        f"floor {int((floor > 0).sum())}")
    np.savez_compressed(os.path.join(out_dir, f"{world}_votes.npz"),
                        wall=wall, obst=obst, floor=floor)
    cv2.imwrite(os.path.join(out_dir, f"{world}_mosaic.png"), mosaic)
    return wall, obst, floor


def wall_mask(wall, other=None):
    """Vote grid -> clean wall mask on the working grid. `other` = votes for
    lower heights at the same cells; a wall cell must be mostly wall-height."""
    k3 = np.ones((3, 3), np.uint8)
    box = np.ones((3, 3), np.float32)
    dense = cv2.filter2D(wall.astype(np.float32), -1, box)
    m = dense >= WALL_MIN_VOTES
    if other is not None:
        m &= dense >= cv2.filter2D(other.astype(np.float32), -1, box)
    m = m.astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    # keep elongated structures: a wall is long; isolated specks are not
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    out = np.zeros_like(m)
    for i in range(1, n):
        ext = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) * RES
        if ext >= 1.0:
            out[lab == i] = 1
    return cv2.dilate(out, k3)


def wall_mask_shape(wall, obst, floor):
    """--shape: walls by SHAPE instead of by a height threshold. Anything
    raised above the floor that is thin (under about 0.45 m across) and long
    (1.5 m or more) is a wall; wall height relative to the camera differs
    between sites, thin-and-long does not."""
    box = np.ones((3, 3), np.float32)
    up = cv2.filter2D((wall + obst).astype(np.float32), -1, box)
    fl = cv2.filter2D(floor.astype(np.float32), -1, box)
    raised = ((up >= WALL_MIN_VOTES) & (up >= fl)).astype(np.uint8)
    raised = cv2.morphologyEx(raised, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    thick = cv2.morphologyEx(raised, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    thin = raised.copy()
    thin[cv2.dilate(thick, np.ones((5, 5), np.uint8)) > 0] = 0
    n, lab, stats, _ = cv2.connectedComponentsWithStats(thin, 8)
    out = np.zeros_like(thin)
    for i in range(1, n):
        ext = max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) * RES
        if ext >= 1.5:
            out[lab == i] = 1
    return cv2.dilate(out, np.ones((3, 3), np.uint8)), raised


def obstacle_mask(obst, wmask):
    dense = cv2.filter2D(obst.astype(np.float32), -1, np.ones((3, 3), np.float32))
    m = (dense >= OBST_MIN_VOTES).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    out = np.zeros_like(m)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] * RES * RES >= 0.05:
            out[lab == i] = 1
    out[cv2.dilate(wmask, np.ones((5, 5), np.uint8)) > 0] = 0
    return out


def to_graded(mask, extent=None):
    """Working-grid mask -> 600x600 graded map centred on the mask's bounding
    box centre (the supervisor centres its ground truth the same way).
    Returns (image uint8 0=wall 255=free, centre_x_m, centre_y_m)."""
    ys, xs = np.nonzero(mask if extent is None else extent)
    if not len(xs):
        return np.full((600, 600), 255, np.uint8), 0.0, 0.0
    if extent is None:
        cxp, cyp = (xs.min() + xs.max()) / 2.0, (ys.min() + ys.max()) / 2.0
    else:
        # The flight rarely passes over the outer walls, so the walls found
        # do not span the building. Everything seen (floor, objects, walls)
        # does: centre on that, trimmed against stray points.
        x_lo, x_hi = np.percentile(xs, [0.5, 99.5])
        y_lo, y_hi = np.percentile(ys, [0.5, 99.5])
        cxp, cyp = (x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0
    x0, y0 = int(round(cxp - 300)), int(round(cyp - 300))
    out = np.full((600, 600), 255, np.uint8)
    sx0, sy0 = max(0, x0), max(0, y0)
    sx1, sy1 = min(GRID, x0 + 600), min(GRID, y0 + 600)
    sub = mask[sy0:sy1, sx0:sx1]
    out[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0][sub > 0] = 0
    return out, (cxp - GRID / 2) * RES, (GRID / 2 - cyp) * RES


def score(world, graded, centre, out_dir, label, log=print):
    """Offline only: compare with the world file."""
    import eval_harness as eh
    gt = eh.load_world(os.path.join(REPO, "worlds", f"{world}_world.wbt"))
    gt_arr, meta = eh.generate_ground_truth_map(gt["walls"])
    g = gt_arr < 128
    e = graded < 128
    iou, f1, acc = eh.iou_f1_acc(e, g)
    b, dx, dy = eh.best_shift(e, g, search=120)
    # tolerant score: a wall cell counts if truth is within 0.2 m, and vice versa
    k = np.ones((9, 9), np.uint8)
    gd = cv2.dilate(g.astype(np.uint8), k) > 0
    ed = cv2.dilate(e.astype(np.uint8), k) > 0
    prec = (e & gd).sum() / max(e.sum(), 1)
    rec = (g & ed).sum() / max(g.sum(), 1)
    tc = (meta["center"][0] - gt["origin"][0], meta["center"][1] - gt["origin"][1])
    log(f"   [{label}] IoU {iou:.3f}  best-shift IoU {b:.3f} at ({dx:+d},{dy:+d}) px  "
        f"F1 {f1:.3f}  | within 0.2 m: precision {prec:.2f} recall {rec:.2f}  | "
        f"wall px est {int(e.sum())} gt {int(g.sum())}"
        + (f"  | centre est ({centre[0]:.2f},{centre[1]:.2f}) true ({tc[0]:.2f},{tc[1]:.2f})"
           if centre is not None else ""))
    vis = np.full((600, 600, 3), 255, np.uint8)
    vis[g & ~e] = (0, 0, 0)
    vis[e & ~g] = (0, 0, 255)
    vis[e & g] = (0, 160, 0)
    cv2.imwrite(os.path.join(out_dir, f"{world}_cmp_{label}.png"), vis)
    return iou, b


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out_dir = (sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv
               else os.path.join(os.path.dirname(REPO), "_scratch", "agentA"))
    if "--out" in sys.argv:
        args = [a for a in args if a != out_dir]
    os.makedirs(out_dir, exist_ok=True)
    for world in args or ["small", "medium", "large"]:
        vp = os.path.join(out_dir, f"{world}_votes.npz")
        if "--reuse" in sys.argv and os.path.exists(vp):
            z = np.load(vp)
            wall, obst, floor = z["wall"], z["obst"], z["floor"]
        else:
            wall, obst, floor = build(world, out_dir)
        wm = wall_mask(wall, obst.astype(np.float32) + floor.astype(np.float32))
        om = obstacle_mask(obst, wm)
        if "--shape" in sys.argv:
            wm, raised = wall_mask_shape(wall, obst, floor)
            om = raised.copy()
            om[cv2.dilate(wm, np.ones((5, 5), np.uint8)) > 0] = 0
        seen = (floor > 0) | (obst > 0) | (wall > 0)
        seen = cv2.morphologyEx(seen.astype(np.uint8), cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        graded, cx, cy = to_graded(wm, seen)
        g2, cx2, cy2 = to_graded(wm)
        print(f"[{world}] centre from all evidence ({cx:.2f},{cy:.2f}), "
              f"from walls only ({cx2:.2f},{cy2:.2f})")
        cv2.imwrite(os.path.join(out_dir, f"{world}_walls.png"),
                    cv2.cvtColor(graded, cv2.COLOR_GRAY2BGR))
        og, _, _ = to_graded(np.maximum(wm, om))   # same framing as walls+obst
        dbg = np.full((GRID, GRID, 3), 255, np.uint8)
        dbg[floor > 0] = (225, 225, 225)
        dbg[om > 0] = (0, 140, 255)
        dbg[wm > 0] = (0, 0, 0)
        cv2.circle(dbg, (GRID // 2, GRID // 2), 6, (0, 200, 0), 2)
        ys, xs = np.nonzero((floor > 0) | (wm > 0) | (om > 0))
        if len(xs):
            dbg = dbg[max(0, ys.min() - 10):ys.max() + 10, max(0, xs.min() - 10):xs.max() + 10]
        cv2.imwrite(os.path.join(out_dir, f"{world}_debug.png"), dbg)
        cv2.imwrite(os.path.join(out_dir, f"{world}_obst.png"), (1 - om) * 255)
        print(f"[{world}] wall cells {int(wm.sum())}, obstacle cells {int(om.sum())} "
              f"({om.sum() * RES * RES:.1f} m2), map centre ({cx:.2f},{cy:.2f}) m")
        if "--noscore" not in sys.argv:
            score(world, graded, (cx, cy), out_dir, "new")
            cur = os.path.join(REPO, "_eval", "pristine", world, "map_estimate.png")
            if os.path.exists(cur):
                c = cv2.imread(cur, cv2.IMREAD_GRAYSCALE)
                score(world, c, None, out_dir, "current")


if __name__ == "__main__":
    main()
