"""Can the flyover wall map be registered onto the startup spin? (Answer: no.)

Motivation. The routing grid starts EMPTY because the flyover wall map is
misaligned by 0.7-1.5 m, and walls that wrong seal real doorways. Discarding a
map that scores 0.843 felt wasteful, so the obvious next idea is to ALIGN it:
the startup spin's lidar walls are registered exactly (the takeoff mat defines
the frame), so cross-correlating the flyover map against them should recover the
offset, after which the map could safely guide routing through real doorways.

This script tests that offline - no Webots run needed. It builds a mat-frame
ground-truth wall grid from the .wbt, fakes the spin by ray-casting 5 m from the
origin (what the robot can actually see before it moves), then runs the very
correlation the controller would run and asks two questions: does a shift win
decisively, and does the winning shift agree better with ground truth?

    .venv/bin/python tools/align_check.py small|medium|large

MEASURED 30/07 - the idea is dead, for two independent reasons:

    world   spin wall cells   best fit          flyover-vs-truth (0.3 m band)
    small   141               rejected (0)      0.258 -> 0.258
    medium   65               rejected (0)      0.062 -> 0.062
    large    44               fit +0.7,-1.8     0.104 -> 0.025  (WORSE)

1. There is almost nothing to register against. Both robots start on a mat in
   the open - outside the house on small, in the truck bay on medium - so the
   spin sees 44-141 wall cells, below any usable threshold.
2. The flyover map is not merely offset. Only 6-26% of its wall cells land
   within 0.3 m of a real wall, so no translation can fix it; on large the
   best-scoring shift made agreement four times worse.

The supervisor's 0.843 map score is an IoU with its own best-shift search and
0.2 m thick walls - generous about exactly the errors that ruin planning. Keep
the routing grid lidar-only; this file is the evidence for that decision.
"""
import sys, math, numpy as np
sys.path.insert(0, "tools"); sys.path.insert(0, "controllers/proposed_solution")
import eval_harness as eh
import prepare_mission as pm

NAV_RES = 0.10
ALIGN_SEARCH_M, ALIGN_ROI_M, ALIGN_MIN_HITS, ALIGN_MIN_GAIN = 2.0, 8.0, 60, 1.35

world = sys.argv[1]
scene = eh.load_world(f"worlds/{world}_world.wbt")
origin = scene["origin"]
N = int(round(2 * pm.GRID_HALF_M / NAV_RES))

def w2c(x, y):
    return int(round(N/2 - y/NAV_RES)), int(round(N/2 + x/NAV_RES))

# ground truth walls in the MAT frame (origin marker at (0,0))
truth = np.zeros((N, N), bool)
for wall in scene["walls"]:
    if wall["pos"][2] > 1.0:
        continue
    cs = eh.wall_corners(wall)
    xs = [c[0]-origin[0] for c in cs]; ys = [c[1]-origin[1] for c in cs]
    for t in np.linspace(0, 1, 400):
        for a, b in ((0,1),(1,2),(2,3),(3,0)):
            x = xs[a] + t*(xs[b]-xs[a]); y = ys[a] + t*(ys[b]-ys[a])
            r, c = w2c(x, y)
            if 0 <= r < N and 0 <= c < N:
                truth[r, c] = True

# fake the spin: cells within 5 m of the origin with line of sight to it
lid = np.zeros((N, N), bool); seen = np.zeros((N, N), bool)
for ang in np.linspace(0, 2*math.pi, 720):
    ca, sa = math.cos(ang), math.sin(ang)
    d = 0.2
    while d < 5.0:
        r, c = w2c(d*ca, d*sa)
        if not (0 <= r < N and 0 <= c < N):
            break
        seen[r, c] = True
        if truth[r, c]:
            lid[r, c] = True
            break
        d += NAV_RES
print(f"{world}: fake spin has {lid.sum()} wall cells, {seen.sum()} seen")

grid = np.load(f"docs/eval/pristine/{world}/nav_grid.npy")
ff = max(1, int(round(NAV_RES / pm.MAP_SCALE))); n2 = grid.shape[0] // ff
fly = (grid[:n2*ff, :n2*ff].reshape(n2, ff, n2, ff).max(axis=(1,3)) > 0)
print("flyover grid", fly.shape, "cells", fly.sum(), "nav N", N)
if fly.shape[0] != N:
    print("!! shape mismatch — alignment would bail out"); sys.exit(1)

roi = int(round(ALIGN_ROI_M/NAV_RES)); pad = int(round(ALIGN_SEARCH_M/NAV_RES))
r0, c0 = w2c(0.0, 0.0)
rlo, rhi = max(pad, r0-roi), min(N-pad, r0+roi)
clo, chi = max(pad, c0-roi), min(N-pad, c0+roi)
L = lid[rlo:rhi, clo:chi]; S = seen[rlo:rhi, clo:chi]; F = S & ~L
best = (0.0, 0, 0)
for dr in range(-pad, pad+1):
    for dc in range(-pad, pad+1):
        f = fly[rlo+dr:rhi+dr, clo+dc:chi+dc]
        s = float(np.count_nonzero(f & L)) - 1.5*float(np.count_nonzero(f & F))
        if s > best[0]:
            best = (s, dr, dc)
null = (float(np.count_nonzero(fly[rlo:rhi, clo:chi] & L))
        - 1.5*float(np.count_nonzero(fly[rlo:rhi, clo:chi] & F)))
s, dr, dc = best
dx, dy = dc*NAV_RES, -dr*NAV_RES
accept = not (s < ALIGN_MIN_HITS or s < ALIGN_MIN_GAIN*max(null, 1.0))
print(f"best fit {s:.0f} at (dx,dy)=({dx:+.2f},{dy:+.2f})  unshifted {null:.0f}  "
      f"=> {'ADOPT' if accept else 'reject'}")

# does the shift actually improve agreement with FULL ground truth?
def agree(f):
    return float(np.count_nonzero(f & truth)) / max(1, np.count_nonzero(f))
sh = np.zeros_like(fly)
src = fly[max(0,dr):N+min(0,dr), max(0,dc):N+min(0,dc)]
sh[max(0,-dr):N+min(0,-dr), max(0,-dc):N+min(0,-dc)] = src
t3 = truth.copy()
for k in range(1, 3):                   # 0.3 m tolerance band on truth
    t3 |= np.roll(truth, k, 0) | np.roll(truth, -k, 0) | np.roll(truth, k, 1) | np.roll(truth, -k, 1)
def agree3(f):
    return float(np.count_nonzero(f & t3)) / max(1, np.count_nonzero(f))
print(f"flyover-vs-truth (0.3 m band): unshifted {agree3(fly):.3f} -> shifted {agree3(sh):.3f}")
