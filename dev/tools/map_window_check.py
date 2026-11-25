"""Which 600x600 window should the learned map deliverable use?

The deliverable is a 30 m x 30 m crop of a much larger grid, and the marking
server compares it against a ground truth centred on the WALL BOUNDING BOX. We
currently centre on the min/max of our own painted cells, which is as
outlier-sensitive as an average of salaries including the CEO: a single fence
post painted out on the paving drags the whole window and the IoU collapses.

Evidence that this is real, not theoretical (small_world, official scores):

    coverage 49%  ->  map 0.808
    coverage 53%  ->  map 0.892
    coverage 64%  ->  map 0.240      <-- more looking, far worse map

A gradual decline would suggest furniture pollution; a collapse suggests the
window moved. This script settles it offline from the saved learned grids, with
no Webots run, by rendering the SAME data under four window rules and scoring
each against ground truth:

    minmax     what we ship now - centre of min/max extent
    pct        centre of the 2nd..98th percentile extent (trim outliers)
    maxmass    the window position containing the most painted wall (integral
               image over all positions - optimal for what IoU rewards)
    component  centre of the largest connected component of painted wall

    .venv/bin/python tools/map_window_check.py small
"""
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "tools")
sys.path.insert(0, "controllers/proposed_solution")
import eval_harness as eh  # noqa: E402

OUT = 600
SIM = "controllers/proposed_solution/sim_logs"


def load_merged(world):
    """Reproduce write_learned_map's merged grid from the saved robot grids."""
    import prepare_mission as pm
    prior_path = f"docs/eval/pristine/{world}/nav_grid.npy"
    grid = np.load(prior_path)
    ff = max(1, int(round(0.10 / pm.MAP_SCALE)))
    n2 = grid.shape[0] // ff
    prior_b = (grid[:n2 * ff, :n2 * ff].reshape(n2, ff, n2, ff).max(axis=(1, 3)) > 0)

    painted = np.zeros(prior_b.shape, bool)
    erased = np.zeros(prior_b.shape, bool)
    seen = np.zeros(prior_b.shape, bool)
    n = 0
    for r in (1, 2):
        p = os.path.join(SIM, f"learned_robot{r}.npz")
        if not os.path.exists(p):
            continue
        d = np.load(p)
        w, s = d["wall"] > 0, d["seen"] > 0
        painted |= w & ~prior_b
        erased |= prior_b & ~w
        seen |= s
        n += 1
    if not n:
        sys.exit("no learned_robot*.npz in sim_logs - run a mission first")
    merged = painted | (prior_b & seen & ~erased)
    return merged, prior_b


def window_minmax(hi):
    ys, xs = np.nonzero(hi)
    return (int(round((xs.min() + xs.max()) / 2)),
            int(round((ys.min() + ys.max()) / 2)))


def window_pct(hi, lo=2, up=98):
    ys, xs = np.nonzero(hi)
    return (int(round((np.percentile(xs, lo) + np.percentile(xs, up)) / 2)),
            int(round((np.percentile(ys, lo) + np.percentile(ys, up)) / 2)))


def window_maxmass(hi):
    """Window position holding the most painted wall, via an integral image."""
    ii = np.pad(np.cumsum(np.cumsum(hi.astype(np.int32), 0), 1), ((1, 0), (1, 0)))
    G = hi.shape[0]
    if G <= OUT:
        return window_minmax(hi)
    step = 4                                   # 0.2 m - finer than the score cares
    best, bxy = -1, (0, 0)
    for y0 in range(0, G - OUT + 1, step):
        for x0 in range(0, G - OUT + 1, step):
            s = (ii[y0 + OUT, x0 + OUT] - ii[y0, x0 + OUT]
                 - ii[y0 + OUT, x0] + ii[y0, x0])
            if s > best:
                best, bxy = s, (x0, y0)
    return (bxy[0] + OUT // 2, bxy[1] + OUT // 2)


def window_component(hi):
    import cv2
    n, lab, stats, cent = cv2.connectedComponentsWithStats(
        hi.astype(np.uint8), connectivity=8)
    if n <= 1:
        return window_minmax(hi)
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (int(round(cent[k][0])), int(round(cent[k][1])))


def render(hi, cx, cy):
    G = hi.shape[0]
    x0 = min(max(cx - OUT // 2, 0), max(G - OUT, 0))
    y0 = min(max(cy - OUT // 2, 0), max(G - OUT, 0))
    crop = hi[y0:y0 + OUT, x0:x0 + OUT]
    out = np.full((OUT, OUT), 255, np.uint8)
    out[crop[:OUT, :OUT] > 0] = 0
    return Image.fromarray(out)


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "small"
    scene = eh.load_world(f"worlds/{world}_world.wbt")
    gt_arr, _ = eh.generate_ground_truth_map(scene["walls"])
    gt = gt_arr < 128

    merged, prior_b = load_merged(world)
    hi = np.kron(merged.astype(np.uint8), np.ones((2, 2), np.uint8))
    print(f"{world}: {int(merged.sum())} merged wall cells "
          f"({int(hi.sum())} px at 0.05 m/px)")

    for name, fn in (("minmax", window_minmax), ("pct", window_pct),
                     ("maxmass", window_maxmass), ("component", window_component)):
        cx, cy = fn(hi)
        img = render(hi, cx, cy)
        est = eh.binarize(img)
        iou, f1, acc = eh.iou_f1_acc(est, gt)
        b_iou, bdx, bdy = eh.best_shift(est, gt, search=60)
        print(f"  {name:<10} centre ({cx},{cy})  IoU {iou:.4f}  F1 {f1:.4f}  "
              f"best-shift IoU {b_iou:.4f} at ({bdx:+d},{bdy:+d}) px")

    # ── content filters ─────────────────────────────────────────────────────
    # The window turns out not to be the problem: minmax reproduces the official
    # 0.24 and no re-framing beats it. So the loss is what we PAINT. Ground truth
    # contains walls only, while lidar cannot tell a wall from a sofa - and the
    # more of a room a robot covers, the more furniture it contributes. Walls are
    # long and straight; furniture is not. Test filters that keep only long
    # straight structure, scored the same way.
    import cv2
    print("  -- content filters (minmax window) --")
    for name, run_px in (("straight-0.5m", 10), ("straight-0.75m", 15),
                         ("straight-1.0m", 20), ("straight-1.5m", 30)):
        h = np.zeros_like(hi)
        for kx, ky in ((run_px, 1), (1, run_px)):
            k = np.ones((ky, kx), np.uint8)
            h |= cv2.morphologyEx(hi, cv2.MORPH_OPEN, k)
        if not h.sum():
            print(f"  {name:<14} empty")
            continue
        # Re-thicken to the ground truth's 0.2 m walls so thin survivors are not
        # penalised for being one pixel wide.
        h = cv2.dilate(h, np.ones((3, 3), np.uint8))
        cx, cy = window_minmax(h)
        est = eh.binarize(render(h, cx, cy))
        iou, f1, _ = eh.iou_f1_acc(est, gt)
        b_iou, bdx, bdy = eh.best_shift(est, gt, search=60)
        print(f"  {name:<14} {int(h.sum()):>6} px  IoU {iou:.4f}  F1 {f1:.4f}  "
              f"best-shift {b_iou:.4f}")

    # For reference: how good is the flyover map we would keep instead?
    fhi = np.kron(prior_b.astype(np.uint8), np.ones((2, 2), np.uint8))
    if fhi.sum():
        cx, cy = window_minmax(fhi)
        est = eh.binarize(render(fhi, cx, cy))
        iou, f1, _ = eh.iou_f1_acc(est, gt)
        print(f"  {'flyover':<10} (reference)          IoU {iou:.4f}  F1 {f1:.4f}")


if __name__ == "__main__":
    main()
