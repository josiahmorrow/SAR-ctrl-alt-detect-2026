"""
Offline extraction A/B: score candidate flyover-analysis variants against
ground truth WITHOUT touching controllers/proposed_solution/sim_logs (so it is
safe to run while Webots batches are using those files) and without Webots.

    .venv/bin/python tools/extract_experiment.py --world medium --mode baseline
    .venv/bin/python tools/extract_experiment.py --world all --mode altscale

Modes
-----
baseline  current pipeline, unmodified.

altscale  physically-motivated scale correction. Visual odometry fixes
          metres-per-pixel at the calibration (mat) altitude, so every pixel
          offset measured while the drone is HIGHER than the mat is
          under-scaled — victims and walls collapse toward the origin. The IMU
          reports altitude (medium: ~0.5 m on the mat, 2.9-3.7 m at cruise),
          so the local linear part of each frame's homography is scaled by
          a_i = alt_i / alt_calib, while the drone's own position keeps the
          radial gain g_i (which preserves the visually-measured bearing).
          Both factors come from the IMU; nothing is fitted to ground truth.
"""
import argparse
import importlib.util
import math
import os
import shutil
import sys
import tempfile

import numpy as np


def _find_env_root(start):
    # dev/ lives inside controllers/proposed_solution of the organisers' repo;
    # the environment root is the first ancestor holding worlds/ and the
    # untouched marking supervisor.
    p = os.path.dirname(os.path.abspath(start))
    while True:
        if (os.path.isdir(os.path.join(p, "worlds")) and os.path.isdir(
                os.path.join(p, "controllers", "sar_marking_supervisor"))):
            return p
        q = os.path.dirname(p)
        if q == p:
            raise RuntimeError("environment root (worlds/ + supervisor) not found")
        p = q

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = _find_env_root(__file__)
PS = os.path.join(REPO, "controllers", "proposed_solution")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "vision_gate_test"))

import cv2  # noqa: F401,E402  (pre-import before the venv shim)
import eval_harness as EH  # noqa: E402

GT_VICTIMS = {}   # filled from the .wbt at run time


def load_pm():
    spec = importlib.util.spec_from_file_location(
        "pm_exp", os.path.join(PS, "prepare_mission.py"))
    pm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pm)
    return pm


def patch_altscale(pm):
    """Replace _imu_scale_align with the altitude-aware variant."""
    orig_integrate = pm._integrate_imu_trajectory

    def aligned(poses, imu_path, fps, total, w_px, h_px):
        pm._LAST_MAP_SCALE = 1.0
        if not imu_path or not os.path.exists(imu_path) or len(poses) < 20:
            return poses
        traj = orig_integrate(imu_path, fps, total, poses[0][0])
        if not traj:
            return poses
        tkeys = np.array(sorted(traj.keys()))

        # Calibration altitude: the IMU altitude at the calibration sample.
        k0 = int(tkeys[np.argmin(np.abs(tkeys - poses[0][0]))])
        h_cal = max(traj[k0][2], 0.15)

        V, I, A, F = [], [], [], []
        for fi, H in poses:
            vx, vy = pm.pix_to_world(w_px / 2.0, h_px / 2.0, H)
            k = int(tkeys[np.argmin(np.abs(tkeys - fi))])
            if abs(k - fi) > fps:
                continue
            ix, iy, ih, _ = traj[k]
            V.append((vx, vy)); I.append((ix, iy)); A.append(ih); F.append(fi)
        if len(V) < 20:
            return poses
        V = np.asarray(V, float); I = np.asarray(I, float)
        A = np.asarray(A, float); F = np.asarray(F, float)

        rV = np.hypot(V[:, 0], V[:, 1]); rI = np.hypot(I[:, 0], I[:, 1])
        g = np.ones(len(V)); m = rV > 0.75
        g[m] = np.clip(rI[m] / rV[m], 1.0, 8.0)
        g = np.array([np.median(g[max(0, i - 4):i + 5]) for i in range(len(g))])

        a = np.clip(A / h_cal, 1.0, 8.0)
        a = np.array([np.median(a[max(0, i - 4):i + 5]) for i in range(len(a))])

        order = np.argsort(F)
        Fs, gs, as_ = F[order], g[order], a[order]
        pm._LAST_MAP_SCALE = float(np.median(gs))
        print(f"[altscale] altitude ratio median x{np.median(as_):.2f} "
              f"(range {as_.min():.2f}-{as_.max():.2f}), radial gain median "
              f"x{np.median(gs):.2f}, h_cal={h_cal:.2f} m")

        out = []
        for fi, H in poses:
            gi = float(np.interp(float(fi), Fs, gs))
            ai = float(np.interp(float(fi), Fs, as_))
            px, py = pm.pix_to_world(w_px / 2.0, h_px / 2.0, H)
            # World-space affine: scale offsets about the drone by ai, and put
            # the drone itself at its radially-corrected position (gi * P).
            M = np.array([[ai, 0.0, (gi - ai) * px],
                          [0.0, ai, (gi - ai) * py],
                          [0.0, 0.0, 1.0]])
            out.append((fi, M @ H))
        return out

    pm._imu_scale_align = aligned


def score(world, csv_path, map_path):
    wbt = os.path.join(REPO, "worlds", f"{world}_world.wbt")
    w = EH.load_world(wbt)
    gt_arr, meta = EH.generate_ground_truth_map(w["walls"])
    origin = np.asarray(w["origin"], float)

    est = []
    with open(csv_path) as fh:
        for ln in fh:
            if ln.strip():
                x, y = ln.split(",")
                est.append((float(x), float(y)))
    est = np.asarray(est, float)

    # victim errors (greedy match, same as the harness)
    gts = [(v["marker_world"][0] - origin[0], v["marker_world"][1] - origin[1])
           for v in w["victims"]]
    e, gl, ds = list(map(tuple, est)), list(gts), []
    while e and gl:
        d, i, j = min((math.hypot(a[0] - b[0], a[1] - b[1]), i, j)
                      for i, a in enumerate(e) for j, b in enumerate(gl))
        ds.append(d); e.pop(i); gl.pop(j)
    mean_err = float(np.mean(ds)) if ds else float("nan")

    from PIL import Image
    est_img = EH.binarize(Image.open(map_path))
    gt_b = gt_arr < 128
    iou, f1, acc = EH.iou_f1_acc(est_img, gt_b)
    return dict(n_est=len(est), n_gt=len(gts), mean_err=mean_err,
                errs=sorted(round(d, 2) for d in ds), missed=len(gl),
                iou=round(iou, 4), f1=round(f1, 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="medium",
                    choices=["small", "medium", "large", "all"])
    ap.add_argument("--mode", default="baseline",
                    choices=["baseline", "altscale"])
    args = ap.parse_args()
    worlds = ["small", "medium", "large"] if args.world == "all" else [args.world]

    for world in worlds:
        pm = load_pm()
        if args.mode == "altscale":
            patch_altscale(pm)
        out = tempfile.mkdtemp(prefix=f"extract_{world}_{args.mode}_")
        pm.SIM_LOGS = out
        pm.CSV_OUT = os.path.join(out, "victim_location_estimates.csv")
        pm.MAP_OUT = os.path.join(out, "map_estimate.png")
        pm._LAST_REC = os.path.join(out, ".last_recording")
        mp4 = os.path.join(REPO, "recordings", f"{world}_world_flyover.mp4")
        imu = mp4.replace(".mp4", ".csv")
        print(f"\n===== {world} / {args.mode} =====", flush=True)
        victims, wall_map = pm.analyse(mp4, imu if os.path.exists(imu) else None)
        pm.save(victims, wall_map)
        res = score(world, pm.CSV_OUT, pm.MAP_OUT)
        print(f"RESULT {world} {args.mode}: est={res['n_est']}/{res['n_gt']} "
              f"mean_err={res['mean_err']:.2f} m errs={res['errs']} "
              f"missed={res['missed']} map_iou={res['iou']}", flush=True)
        shutil.rmtree(out, ignore_errors=True)


if __name__ == "__main__":
    main()
