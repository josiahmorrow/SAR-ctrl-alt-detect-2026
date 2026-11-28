"""Offline: render the pristine nav_grid for a world with a robot trajectory
(from a run log) and the video waypoints. usage: showgrid.py <world> <robot log> <out.png>"""
import re, sys
import numpy as np, cv2

world, log, out = sys.argv[1:4]
g = np.load(rf"C:\Users\agcoo\projects\sar\sar-env\_eval\pristine\{world}\nav_grid.npy",
            allow_pickle=True)
print("grid", g.shape, g.dtype, np.unique(g)[:10])
N = g.shape[0]
RES = 0.05
img = np.where(g > 0, 0, 255).astype(np.uint8)
img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
TEL = re.compile(r"t=\s*([\d.]+) \w+\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)\s+wp\d+\s*=\(\s*([-\d.]+),\s*([-\d.]+)\)")
pts, wps = [], set()
for line in open(log, encoding="utf-8", errors="replace"):
    m = TEL.search(line)
    if m:
        pts.append((float(m.group(2)), float(m.group(3))))
        wps.add((float(m.group(4)), float(m.group(5))))


def w2c(x, y):  # assume origin at grid centre, +y up
    return int(round(N / 2 + x / RES)), int(round(N / 2 - y / RES))


for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
    cv2.line(img, w2c(x0, y0), w2c(x1, y1), (255, 0, 0), 1)
for x, y in wps:
    cv2.circle(img, w2c(x, y), 4, (0, 0, 255), -1)
ys, xs = np.nonzero(g > 0)
if len(xs):
    x0, x1, y0, y1 = max(0, xs.min() - 20), xs.max() + 20, max(0, ys.min() - 20), ys.max() + 20
    img = img[y0:y1, x0:x1]
img = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(out, img)
print("wrote", out, img.shape)
