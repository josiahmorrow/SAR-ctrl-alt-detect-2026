"""Offline: fine-texture measures on snapshot frames.
usage: texcheck.py <snap_dir> <run_dir> [label_spec...]
label_spec: robotN:t0:t1:LABEL  (frames of robotN with t0<=t<=t1 get LABEL)
For each labelled frame: depth (from the run log), blob area, std S, std V,
and fine texture = var(Laplacian(V)) inside the eroded blob / mean(V)^2,
plus high-pass energy = mean |V - blur5(V)| / mean V.
"""
import glob, os, re, sys
import cv2, numpy as np

snap, run = sys.argv[1], sys.argv[2]
specs = []
for s in sys.argv[3:]:
    r, a, b, lab = s.split(":")
    specs.append((r, float(a), float(b), lab))

TEL = re.compile(r"^\[(robot\d)\] t=\s*([\d.]+) .*?depth=\s*([-\d.inf]+)")
tel = {}
for lf in glob.glob(os.path.join(run, "*_tee", "proposed_solution_*.log")):
    for l in open(lf, encoding="utf-8", errors="replace"):
        m = TEL.search(l)
        if m:
            tel.setdefault(m.group(1), []).append((float(m.group(2)), float(m.group(3))))


def depth_at(rid, t):
    best = None
    for tt, d in tel.get(rid, []):
        if best is None or abs(tt - t) < abs(best[0] - t):
            best = (tt, d)
    return best[1] if best and abs(best[0] - t) < 0.6 else float("nan")


rows = []
for f in sorted(glob.glob(os.path.join(snap, "robot*_*.png"))):
    if f.endswith("_blob.png"):
        continue
    base = os.path.basename(f)[:-4]
    rid, ts = base.split("_")[0], float(base.split("_")[1])
    lab = None
    for r, a, b, L in specs:
        if r == rid and a <= ts <= b:
            lab = L
    if lab is None:
        continue
    img = cv2.imread(f)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    denim = cv2.inRange(hsv, np.array([95, 35, 25]), np.array([135, 255, 255]))
    denim = cv2.morphologyEx(denim, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(denim, 8)
    if n <= 1:
        continue
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    blob = (labels == best).astype(np.uint8)
    er = cv2.erode(blob, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))).astype(bool)
    if er.sum() < 200:
        continue
    V = hsv[:, :, 2].astype(np.float32)
    S = hsv[:, :, 1].astype(np.float32)
    mv = float(V[er].mean())
    lap = cv2.Laplacian(V, cv2.CV_32F, ksize=3)
    fine = float(lap[er].var()) / max(mv, 1.0) ** 2
    hp = float(np.abs(V - cv2.GaussianBlur(V, (5, 5), 0))[er].mean()) / max(mv, 1.0)
    rows.append((lab, rid, ts, depth_at(rid, ts), blob.sum() / blob.size,
                 float(S[blob.astype(bool)].std()), float(V[blob.astype(bool)].std()),
                 fine, hp))

for r in rows:
    print(f"{r[0]:8s} {r[1]} t={r[2]:6.1f} d={r[3]:5.2f} blob={r[4]:.3f} "
          f"texs={r[5]:5.1f} texv={r[6]:5.1f} fine={r[7]:.4f} hp={r[8]:.4f}")
print()
for lab in sorted(set(r[0] for r in rows)):
    for dmax in (0.9, 99):
        sel = [r for r in rows if r[0] == lab and (r[3] <= dmax)]
        if not sel:
            continue
        f = np.array([r[7] for r in sel]); h = np.array([r[8] for r in sel])
        print(f"{lab:8s} d<={dmax:<4} n={len(sel):3d} fine min/med/max "
              f"{f.min():.4f}/{np.median(f):.4f}/{f.max():.4f}  hp "
              f"{h.min():.4f}/{np.median(h):.4f}/{h.max():.4f}")
