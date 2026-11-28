"""Dense wall map from the flyover: near-nadir orthomosaic, then wall bands.

OFFLINE development tool (second approach; the first, flyover_map.py, used
sparse feature parallax and found too little of each wall).

Stage 1 (--build): with the dead-reckoned poses (flyover_map.poses, i.e. the
flyover_dr method), every ground cell takes its colour from the frame in
which the camera was most nearly overhead. A point at height h seen at
ground distance d from the nadir is displaced by d*h/(H-h); at the nadir
the displacement is zero, so wall TOPS land in their true plan position
where the cell was seen from close to overhead. Stored per cell: colour,
nadir distance, number of views.

Stage 2: walls are long, thin, straight bands in that image. See
segment_walls().

    python -X utf8 dev/tools/flyover_ortho.py small medium large --build
    python -X utf8 dev/tools/flyover_ortho.py small medium large

Ground truth (eval_harness) is used for scoring only.
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flyover_map as fm                 # noqa: E402  (poses, scoring; read only)

REPO = fm.REPO
RES = 0.05                  # m per cell (graded resolution)
GRID = 1200                 # 60 m working grid centred on the origin
R_MAX_M = 2.6               # farthest nadir distance a cell may be taken from
FRAME_STEP = 3
OUT = os.path.join(os.path.dirname(REPO), "_scratch", "agentA2")


def build(world, log=print):
    vid = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
    imu = os.path.join(REPO, "recordings", f"{world}_world_flyover.csv")
    P = fm.poses(vid, imu, log)
    n, p, O, s, r = P["n"], P["p"], P["O"], P["s"], P["r"]
    half = int(round(R_MAX_M / RES))
    side = 2 * half + 1
    yy, xx = np.mgrid[-half:half + 1, -half:half + 1]
    dist = (np.hypot(xx, yy) * RES).astype(np.float32)
    ortho = np.zeros((GRID, GRID, 3), np.uint8)
    best = np.full((GRID, GRID), np.inf, np.float32)
    views = np.zeros((GRID, GRID), np.uint16)
    cap = cv2.VideoCapture(vid)
    used = 0
    for i in range(n):
        if not cap.grab():
            break
        if i < r or i % FRAME_STEP:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        h, w = frame.shape[:2]
        # ROI cell (u, v), u right, v down, centred on the camera's cell
        cc = int(round(p[i][0] / RES + GRID / 2))
        cr = int(round(GRID / 2 - p[i][1] / RES))
        if not (half <= cc < GRID - half and half <= cr < GRID - half):
            continue
        # ground offset of ROI cell from the camera: (dx, dy) = ((u-half)*RES + ex, -(v-half)*RES + ey)
        ex = (cc - GRID / 2) * RES - p[i][0]
        ey = (GRID / 2 - cr) * RES - p[i][1]
        Oi_inv = np.linalg.inv(O(i)) / s          # ground offset -> pixel offset
        A = Oi_inv @ np.array([[RES, 0.0], [0.0, -RES]])
        b = Oi_inv @ np.array([-half * RES + ex, half * RES + ey]) + np.array([w / 2.0, h / 2.0])
        M = np.hstack([A, b.reshape(2, 1)]).astype(np.float64)   # ROI cell -> pixel
        patch = cv2.warpAffine(frame, M, (side, side),
                               flags=cv2.INTER_AREA | cv2.WARP_INVERSE_MAP,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        # validity: the source pixel lies inside the frame
        px = A[0, 0] * (xx + half) + A[0, 1] * (yy + half) + b[0]
        py = A[1, 0] * (xx + half) + A[1, 1] * (yy + half) + b[1]
        valid = (px >= 2) & (px < w - 2) & (py >= 2) & (py < h - 2) & (dist <= R_MAX_M)
        sl = (slice(cr - half, cr + half + 1), slice(cc - half, cc + half + 1))
        views[sl][valid] += 1
        better = valid & (dist < best[sl])
        ortho[sl][better] = patch[better]
        best[sl][better] = dist[better]
        used += 1
    cap.release()
    log(f"[{world}] orthomosaic from {used} frames, {s:.4f} m/px, "
        f"cells seen {(views > 0).sum()}, median nadir distance "
        f"{np.median(best[np.isfinite(best)]):.2f} m")
    os.makedirs(OUT, exist_ok=True)
    np.savez_compressed(os.path.join(OUT, f"{world}_ortho.npz"),
                        ortho=ortho, best=best, views=views, path=p[r::10])
    return ortho, best, views


def crop_box(mask, pad=10):
    ys, xs = np.nonzero(mask)
    return (max(0, ys.min() - pad), min(GRID, ys.max() + pad),
            max(0, xs.min() - pad), min(GRID, xs.max() + pad))


MIN_SEG_M = 1.5             # shortest raw straight edge kept
MIN_WALL_M = 2.0            # shortest merged wall line kept
GAP_M = float(os.environ.get('ORTHO_GAP', '0.5'))                 # collinear pieces closer than this are one line
LINE_TOL_M = 0.10           # pieces within this of one line are collinear
ANGLE_TOL_DEG = 4.0         # tolerance to the building's two directions
WALL_THICK_M = 0.20         # wall thickness (Webots Wall default; ASSUMED)
BAND_NEAR_M = -0.05          # band drawn from here (camera side negative)...
BAND_FAR_M = 0.25           # ...to here beyond the detected base edge
SEAM_MIN_M = 0.5            # nadir distance above which a ridge is a seam
RIM_M = 0.25                # width of the coverage rim ignored for edges


def edge_map(ortho, best, views):
    """Edges of the orthomosaic that are not artefacts of how it was built
    (coverage boundary, seams between views taken from different legs)."""
    lab = cv2.cvtColor(cv2.medianBlur(ortho, 5), cv2.COLOR_BGR2LAB)
    chans = list(cv2.split(lab))
    # local contrast normalisation: the scenes differ a lot in brightness
    chans[0] = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16)).apply(chans[0])
    e = np.zeros(ortho.shape[:2], np.uint8)
    grad = np.zeros(ortho.shape[:2], np.float32)
    for ch, (lo, hi) in zip(chans, ((25, 60), (12, 30), (12, 30))):
        e |= cv2.Canny(ch, lo, hi)
        gx = cv2.Sobel(ch, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(ch, cv2.CV_32F, 0, 1, ksize=3)
        grad = np.maximum(grad, cv2.magnitude(gx, gy) * (1.0 if ch is chans[0] else 2.0))
    seen = (views > 0).astype(np.uint8)
    # the rim of the coverage (cells only ever seen from R_MAX away) is an
    # artefact, not a wall; elsewhere edges may run right up to unseen cells
    inner = cv2.erode(seen, np.ones((3, 3), np.uint8))
    inner[np.where(np.isfinite(best), best, 9.0) > R_MAX_M - RIM_M] = 0
    b = np.where(np.isfinite(best), best, 0).astype(np.float32)
    ridge = (b >= cv2.dilate(b, np.ones((5, 5), np.uint8)) - 1e-4) & (b > SEAM_MIN_M)
    seam = cv2.dilate(ridge.astype(np.uint8), np.ones((5, 5), np.uint8))
    bad = (inner == 0) | (seam > 0)
    e[bad] = 0
    grad[bad] = 0
    return e, seam, grad


def dominant_angle(segs):
    """Building direction in degrees, modulo 90, weighted by length."""
    a = np.degrees(np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])) % 90.0
    ln = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    h, edges = np.histogram(a, bins=90, range=(0, 90), weights=ln)
    h = h + np.roll(h, 1) + np.roll(h, -1)
    k = int(np.argmax(h))
    sel = np.abs(((a - (k + 0.5) + 45) % 90) - 45) < 3
    # circular mean around the peak
    d = ((a[sel] - (k + 0.5) + 45) % 90) - 45
    return float((k + 0.5 + np.average(d, weights=ln[sel])) % 90.0)


def merge_lines(segs, theta_deg):
    """Rotate so the building is axis-aligned, then merge collinear pieces.
    Returns [(axis, c, a0, a1)] in rotated cell coordinates: axis 0 = line
    along x at y=c, axis 1 = line along y at x=c."""
    th = np.radians(theta_deg)
    R = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])   # rotate by -theta
    out = []
    P0 = segs[:, :2] @ R.T
    P1 = segs[:, 2:] @ R.T
    ang = np.degrees(np.arctan2(P1[:, 1] - P0[:, 1], P1[:, 0] - P0[:, 0])) % 180.0
    for axis in (0, 1):
        want = 0.0 if axis == 0 else 90.0
        sel = np.abs(((ang - want + 90) % 180) - 90) < ANGLE_TOL_DEG
        if not sel.any():
            continue
        along = 0 if axis == 0 else 1
        across = 1 - along
        items = sorted(((0.5 * (p0[across] + p1[across]), min(p0[along], p1[along]),
                         max(p0[along], p1[along])) for p0, p1 in zip(P0[sel], P1[sel])))
        tol = LINE_TOL_M / RES
        groups = []
        for c, a0, a1 in items:
            if groups and abs(c - groups[-1]["c"]) <= tol:
                g = groups[-1]
                g["iv"].append((a0, a1))
                g["cs"].append(c)
                g["c"] = float(np.mean(g["cs"]))
            else:
                groups.append({"c": c, "cs": [c], "iv": [(a0, a1)]})
        for g in groups:
            iv = sorted(g["iv"])
            cur0, cur1 = iv[0]
            merged = []
            for a0, a1 in iv[1:]:
                if a0 - cur1 <= GAP_M / RES:
                    cur1 = max(cur1, a1)
                else:
                    merged.append((cur0, cur1))
                    cur0, cur1 = a0, a1
            merged.append((cur0, cur1))
            for a0, a1 in merged:
                if (a1 - a0) * RES >= MIN_WALL_M:
                    out.append((axis, g["c"], a0, a1))
    return out, R


EXTEND = '--noextend' not in sys.argv
EXT_FRAC = 0.5              # a line grows while the edge keeps half its strength
FURN_MIN_CELLS = 100        # voted cells needed in the strip to judge it
FURN_LOW_FRAC = 0.6         # share of low-raised votes that marks a table top


def segment_walls(ortho, best, views, path, votes=None, log=print):
    e, seam, grad = edge_map(ortho, best, views)
    segs = cv2.HoughLinesP(e, 1, np.pi / 360, threshold=25,
                           minLineLength=int(MIN_SEG_M / RES), maxLineGap=6)
    mask = np.zeros(e.shape, np.uint8)
    if segs is None:
        return mask, e, []
    segs = segs.reshape(-1, 4).astype(np.float64)
    theta = dominant_angle(segs)
    lines, R = merge_lines(segs, theta)
    Rinv = R.T

    def strength(axis, c, a):
        # edge strength across the line at along-position a (rotated frame)
        along = 0 if axis == 0 else 1
        bestv = 0.0
        for off in (-2, -1, 0, 1, 2):
            pr = np.zeros(2)
            pr[along], pr[1 - along] = a, c + off
            x, y = Rinv @ pr
            xi, yi = int(round(x)), int(round(y))
            if 0 <= xi < GRID and 0 <= yi < GRID:
                bestv = max(bestv, float(grad[yi, xi]))
        return bestv

    if EXTEND:
        ext = []
        for axis, c, a0, a1 in lines:
            prof = [strength(axis, c, a) for a in np.arange(a0, a1, 1.0)]
            base = float(np.median(prof)) if prof else 0.0
            lo = EXT_FRAC * base
            for direction in (1, -1):
                a = a1 if direction > 0 else a0
                while True:
                    nxt = [strength(axis, c, a + direction * k) for k in range(1, 11)]
                    if base <= 0 or np.median(nxt) < lo:
                        break
                    a += direction * 5
                    if abs(a) > 3 * GRID:
                        break
                if direction > 0:
                    a1 = a
                else:
                    a0 = a
            ext.append((axis, c, a0, a1))
        lines = ext
    # camera path in cell coordinates
    pc = np.stack([path[:, 0] / RES + GRID / 2, GRID / 2 - path[:, 1] / RES], axis=1)
    thick = WALL_THICK_M / RES
    step = 0.5 / RES
    drawn = []
    for axis, c, a0, a1 in lines:
        along = 0 if axis == 0 else 1
        if votes is not None:
            # Furniture test: an edge that bounds a flat raised surface wider
            # than a wall (a table or bench top) is not a wall. The first
            # tool's parallax votes say what lies just beyond the edge.
            reject = False
            stats = []
            for sgn in (1.0, -1.0):
                q = []
                for a, off in ((a0, 0.15), (a1, 0.15), (a1, 0.75), (a0, 0.75)):
                    pr = np.zeros(2)
                    pr[along], pr[1 - along] = a, c + sgn * off / RES
                    q.append(Rinv @ pr)
                strip = np.zeros(e.shape, np.uint8)
                cv2.fillPoly(strip, [np.round(np.array(q)).astype(np.int32)], 1)
                st = strip > 0
                n_low = int(((votes["obst"] > 0) & st).sum())
                n_wall = int(((votes["wall"] > 0) & st).sum())
                n_floor = int(((votes["floor"] > 0) & st).sum())
                stats.append((n_low, n_wall, n_floor))
                tot = n_low + n_wall + n_floor
                if tot >= FURN_MIN_CELLS and n_low >= FURN_LOW_FRAC * tot \
                        and n_wall <= 0.03 * tot:
                    reject = True
            if "--lines" in sys.argv:
                log(f"      line axis {axis} c {c:.0f} len {(a1 - a0) * RES:.1f} m "
                    f"low/wall/floor each side {stats} {'FURNITURE' if reject else ''}")
            if reject:
                continue
        k = max(1, int(round((a1 - a0) / step)))
        for j in range(k):
            s0 = a0 + (a1 - a0) * j / k
            s1 = a0 + (a1 - a0) * (j + 1) / k
            mid_r = np.zeros(2)
            mid_r[along], mid_r[1 - along] = 0.5 * (s0 + s1), c
            mid = Rinv @ mid_r
            near = pc[np.argmin(((pc - mid) ** 2).sum(1))]
            near_r = R @ near
            side = 1.0 if near_r[1 - along] < c else -1.0    # wall is away from the camera
            q = []
            n_, f_ = side * BAND_NEAR_M / RES, side * BAND_FAR_M / RES
            for a, off in ((s0, n_), (s1, n_), (s1, f_), (s0, f_)):
                pr = np.zeros(2)
                pr[along], pr[1 - along] = a, c + off
                q.append(Rinv @ pr)
            cv2.fillPoly(mask, [np.round(np.array(q)).astype(np.int32)], 1)
        drawn.append((axis, c, a0, a1))
    log(f"   building direction {theta:.1f} deg, {len(segs)} raw edges, "
        f"{len(lines)} wall lines, {int(mask.sum())} wall cells")
    return mask, e, lines


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    os.makedirs(OUT, exist_ok=True)
    for world in args or ["small", "medium", "large"]:
        zp = os.path.join(OUT, f"{world}_ortho.npz")
        if "--build" in sys.argv or not os.path.exists(zp):
            ortho, best, views = build(world)
        else:
            z = np.load(zp)
            ortho, best, views = z["ortho"], z["best"], z["views"]
        path = np.load(zp)["path"]
        print(f"[{world}]")
        vp = os.path.join(os.path.dirname(OUT), "agentA", f"{world}_votes.npz")
        votes = None
        if os.path.exists(vp) and "--votes" in sys.argv:
            vz = np.load(vp)
            votes = {k: vz[k] for k in ("wall", "obst", "floor")}
        wm, e, lines = segment_walls(ortho, best, views, path, votes)
        graded, cx, cy = fm.to_graded(wm)
        cv2.imwrite(os.path.join(OUT, f"{world}_walls.png"),
                    cv2.cvtColor(graded, cv2.COLOR_GRAY2BGR))
        yy0, yy1, xx0, xx1 = crop_box(views > 0, pad=30)
        dbg = ortho.copy()
        dbg[e > 0] = (0, 255, 255)
        dbg[wm > 0] = (0, 0, 255)
        cv2.imwrite(os.path.join(OUT, f"{world}_wallsdbg.png"),
                    cv2.resize(dbg[yy0:yy1, xx0:xx1], None, fx=2, fy=2,
                               interpolation=cv2.INTER_NEAREST))
        # free-space prior for the robots (working grid, origin at GRID/2):
        # 0 unseen, 1 seen and no wall found, 2 wall, 3 raised (parallax votes)
        prior = np.zeros(wm.shape, np.uint8)
        prior[np.isfinite(best) & (best <= R_MAX_M - RIM_M)] = 1
        if os.path.exists(vp):
            vz2 = np.load(vp)
            raised = ((vz2["obst"] + vz2["wall"]) >= 3) & ((vz2["obst"] + vz2["wall"]) >= vz2["floor"])
            prior[cv2.dilate(raised.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0] = 3
        prior[wm > 0] = 2
        np.savez_compressed(os.path.join(OUT, f"{world}_prior.npz"), prior=prior,
                            res=RES, origin_cell=GRID // 2)
        pv = np.zeros(prior.shape + (3,), np.uint8)
        pv[prior == 1] = (235, 235, 235)
        pv[prior == 3] = (0, 140, 255)
        pv[prior == 2] = (0, 0, 0)
        cv2.imwrite(os.path.join(OUT, f"{world}_prior.png"), pv[yy0:yy1, xx0:xx1])
        print(f"   prior: seen {(prior > 0).sum() * RES * RES:.0f} m2, wall cells "
              f"{int((prior == 2).sum())}, raised {(prior == 3).sum() * RES * RES:.0f} m2")
        if "--noscore" not in sys.argv:
            fm.score(world, graded, (cx, cy), OUT, "ortho")
            cur = os.path.join(REPO, "_eval", "pristine", world, "map_estimate.png")
            if os.path.exists(cur):
                fm.score(world, cv2.imread(cur, cv2.IMREAD_GRAYSCALE), None, OUT, "current")
        y0, y1, x0, x1 = crop_box(views > 0)
        cv2.imwrite(os.path.join(OUT, f"{world}_ortho.png"), ortho[y0:y1, x0:x1])
        nd = np.where(np.isfinite(best), np.clip(best / R_MAX_M * 255, 0, 255), 255).astype(np.uint8)
        cv2.imwrite(os.path.join(OUT, f"{world}_nadir.png"), nd[y0:y1, x0:x1])


if __name__ == "__main__":
    main()
