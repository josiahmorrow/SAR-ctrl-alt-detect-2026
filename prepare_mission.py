"""
Offline flyover-video analysis for IEEE SMCS SAR Competition (Phase 1).

Usage (run from any directory):
    python prepare_mission.py --video recordings/small_world_flyover.mp4
    python prepare_mission.py --video recordings/medium_world_flyover.mp4 --imu recordings/medium_world_flyover.csv

Outputs written to controllers/proposed_solution/sim_logs/:
    victim_location_estimates.csv  —  one "x,y" per line (metres, world frame)
    map_estimate.png               —  600x600 binary PNG  (0,0,0)=wall  (255,255,255)=free

Improvements over v1:
  - Recording matched to the currently-open world via .wbt access time
  - Cache keyed to recording path so switching worlds forces fresh analysis
  - ArUco calibration scans the full video and keeps the best frame found
  - Map built from Otsu-thresholded top-down mosaic (dark=wall, light=floor)
    instead of Hough lines which produced empty maps on simulation footage
  - Victim size bounds scaled to estimated flight altitude so detection holds
    at varying drone heights; circularity filter reduces furniture false-positives
"""

import argparse
import csv as _csv_mod
import glob
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image

# Reproducibility: ORB/RANSAC draw on OpenCV's RNG, so an unseeded pipeline
# gives slightly different poses — and therefore different victim estimates —
# on each run. Observed 30/07: the same world analysed in two checkouts put a
# victim report 0.2 m apart. Seeding makes the graded deliverables a
# deterministic function of the recording, which is what lets us A/B the
# controller without extraction noise underneath it.
cv2.setRNGSeed(20260730)
np.random.seed(20260730)

# ── Output paths ──────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_LOGS    = os.path.join(_SCRIPT_DIR, "sim_logs")
CSV_OUT     = os.path.join(SIM_LOGS, "victim_location_estimates.csv")
MAP_OUT     = os.path.join(SIM_LOGS, "map_estimate.png")
_LAST_REC   = os.path.join(SIM_LOGS, ".last_recording")  # cache key

# ── World / map constants ─────────────────────────────────────────────────────
MAP_PX    = 600
MAP_SCALE = 0.05          # metres per pixel → 30 m × 30 m total
HALF_M    = MAP_PX * MAP_SCALE / 2.0   # 15 m

# ── ArUco config ──────────────────────────────────────────────────────────────
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.25

# ── Victim detection ──────────────────────────────────────────────────────────
HUMAN_DIAM_M      = 0.55   # expected blob diameter of a standing human from above (m)
HUMAN_AREA_MIN_F  = 0.015  # accept blobs ≥ 1.5% of expected human area
HUMAN_AREA_MAX_F  = 14.0   # accept blobs up to 14× expected area
CIRCULARITY_MIN   = 0.10   # allow non-circular silhouettes (person lying flat, etc.)
CONVEXITY_MIN     = 0.55   # reject obviously fragmented blobs (re-entrant contours)
ORIGIN_EXCL_M     = 3.5    # ignore detections within 3.5 m of origin (mat + near-mat clutter)
CLUSTER_R_M       = 1.5    # merge radius for clustering (wider to merge victim sub-clusters)
MIN_CLUSTER_VOTES = 1      # minimum votes per cluster (1 because victims may appear in only one frame)
DIVERSITY_R_M     = 2.0    # min distance between two selected victim positions (ensures spread)
EXTRA_SLOT_SEP_M  = 3.0    # 5th/6th estimates must be a genuinely new area, not
                           # a near-duplicate of an already-covered victim
DENIM_CLUSTER_R   = 0.8    # tighter merge radius for the precise denim detector
DENIM_DIVERSITY_R = 1.1    # victims can lie 1.4 m apart (small_world v1/v2 do)
MAX_VICTIM_ESTIMATES = 6   # headroom for large_world's 5 GT victims + 1 spare; a vote-based
                           # quality floor (see cluster_detections) gates slots beyond the old
                           # cap of 4 so a correctly-sized world can't gain a spurious estimate
DET_MAX_RANGE_M   = 3.0    # reject detections >3 m from drone (long-range homography errors)

# ── Camera calibration (chessboard on origin mat) ─────────────────────────────
CHESSBOARD_INNER = (8, 5)   # inner corners: 9×6 board  -> (9-1)×(6-1)
CHESSBOARD_SQ_M  = 0.045    # tile side = 45 mm

# ── Vision pose tracking (v5) ─────────────────────────────────────────────────
TRACK_HZ         = 4       # tracking samples per second of video
MIN_STEP_INLIERS = 10      # min RANSAC inliers for a valid frame-to-frame step
STEP_SCALE_MIN   = 0.55    # reject similarity steps with implausible scale change
STEP_SCALE_MAX   = 1.8
MAX_BRIDGE       = 12      # bridge up to N samples (3 s) over failed steps
MIN_TRACK_COVER  = 0.5     # fall back to IMU when < 50% of samples get a vision pose
LC_MIN_GAP       = 20      # loop closure: only match samples >= N samples apart
LC_RADIUS_M      = 3.0     # loop closure: candidate search radius (metres)
LC_RADIUS_DIVE   = 8.0     # wider search for dive samples (pose badly off there)
LC_MIN_INL       = 20      # loop closure: stricter inlier gate than chain steps
LC_ITERS         = 3       # loop closure: re-match/re-solve iterations

# ── Map building (v5) ─────────────────────────────────────────────────────────
# Intermediate world grid: 60 m x 60 m so every dev world fits regardless of
# where the origin mat sits (medium_world walls reach ~(+20, -27) in the
# origin/ArUco frame).  Final 600x600 output is cropped from this grid, centred
# on the detected walls' bounding box — matching how the marking supervisor
# centres its ground-truth map.
GRID_PX     = 1200
GRID_HALF_M = GRID_PX * MAP_SCALE / 2.0   # 30 m
CROP_MARGIN_F = 0.12       # ignore outer 12% of each frame when warping into map
WALL_THICK_PX = 4          # walls are 0.2 m => 4 px at 0.05 m/px

# ── PASS 1.5: pose refinement against the accumulated mosaic (v6) ─────────────
# Frame-to-frame chaining leaves ~1-2 m of local warp (worst at victim dives,
# where it directly biases the victim CSV).  After the chain/anchor solve,
# every posed sample is re-registered against the consensus mean mosaic —
# map-to-frame instead of frame-to-frame — which corrects local drift against
# a fixed reference instead of compounding it.
REF_ITERS       = int(os.environ.get("SAR_REF_ITERS", "2"))   # refinement iterations (0 disables)
REF_SCALE       = 0.10   # refinement grid resolution (m/px) — coarse is robust
REF_PX          = 600    # 60 m x 60 m at 0.10 m/px
REF_SEARCH_M    = 1.6    # translation search radius (cruise)
REF_SEARCH_DIVE = 3.0    # translation search radius (dive samples: worse poses).
                         # Widening this to 10.0 was tested and made no difference —
                         # dive matches land at (0,0) shift with high NCC confidence
                         # regardless of search radius, because a close-up dive frame
                         # is the wrong *kind* of content to match against a cruise-
                         # altitude mosaic (different scale/appearance), not because
                         # the true offset is outside the window. Fixing dive-frame
                         # pose accuracy needs a different mechanism (e.g. IMU-based
                         # dead-reckoning across the dive, or tighter anchoring to the
                         # cruise frames immediately before/after it), not a wider search.
REF_MIN_RESP    = 0.30   # min masked-NCC peak to accept a correction
REF_DAMP        = 0.85   # apply this fraction of the measured correction

# ── Wall extraction from the mosaic (v6) ──────────────────────────────────────
# Walls are 3 m tall: their parallax smear is view-dependent, so mean-minus-min
# ("spread") highlights them as thick bands centred near the wall line, while
# flat floor/terrain cancels out.  Long axis-aligned walls integrate into
# strong peaks in the column/row projections of the spread image even where
# the signal is locally noisy.
SEG_REL_THR   = 0.18     # projection peak floor, relative to strongest peak (was
                         # 0.30 — exterior walls are farther from the flight
                         # path and their smear is weaker, so a high floor
                         # dropped them entirely and shrank the wall bbox used
                         # for centring; see docs/optimisation.md 07-12 log)
SEG_BAND      = 4        # half-width (px) of the lane sampled around a peak
SEG_RUN_THR_F = 0.40     # per-lane support threshold vs lane p90
SEG_GAP_PX    = 20       # bridge support gaps up to 1 m (doorways score as wall)
SEG_MIN_RUN   = 30       # minimum wall segment length: 1.5 m
SEG_MIN_DENS  = 0.35     # minimum on-fraction within a run (was 0.45 — same
                         # exterior-completeness reasoning as SEG_REL_THR)
SEG_NUDGE_PX  = 6        # exterior walls: smear bias is outward — nudge inward
SEG_SNAP_PX   = 14       # snap segment ends to perpendicular walls within 0.7 m
SEG_BBOX_PAD  = 10       # allow lines this far outside the strong-line bbox

SAR_DEBUG = bool(os.environ.get("SAR_DEBUG"))

# Module-level: set by analyse() so save() can write the cache key
_current_recording: str = ""


# ══════════════════════════════════════════════════════════════════════════════
#  Recording selection
# ══════════════════════════════════════════════════════════════════════════════

def find_recording(world_path: str = None) -> tuple:
    """
    Return (mp4_path, imu_path_or_None) for the recording that matches the
    currently-open Webots world.

    Strategy (ordered; each step must be SAFE on unseen judging worlds):
      1. Exact match <world_stem>_flyover.mp4 — the documented convention.
      2. Loose match: any recording whose filename contains the world stem
         (covers <stem>.mp4, flyover_<stem>.mp4 and similar variants).
      3. If exactly ONE recording exists, use it — a single-world judging
         setup can only mean that one.
      4. Otherwise return None: run the mission on lidar alone.

    Deliberately NO "newest mp4" / world-file-timestamp guessing. Analysing
    the WRONG world's flyover is worse than analysing none: the victim CSV and
    map deliverables are graded, so a confident answer from another building
    scores near zero AND sends the robots to phantom coordinates, whereas with
    no flyover the mission still runs (sweep + lidar + camera) and the map
    deliverable is still produced from the robots' own lidar.
    """
    repo_root      = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", ".."))
    recordings_dir = os.path.join(repo_root, "recordings")

    def _with_imu(mp4):
        imu = os.path.splitext(mp4)[0] + ".csv"
        return mp4, (imu if os.path.exists(imu) else None)

    mp4s = sorted(glob.glob(os.path.join(recordings_dir, "*.mp4")))

    if world_path:
        stem = os.path.splitext(os.path.basename(world_path))[0]
        exact = os.path.join(recordings_dir, f"{stem}_flyover.mp4")
        if os.path.exists(exact):
            print(f"[preflight] World '{stem}' (from Robot.getWorldPath()) -> "
                  f"{os.path.basename(exact)}")
            return _with_imu(exact)
        loose = [m for m in mp4s if stem in os.path.basename(m)]
        if len(loose) == 1:
            print(f"[preflight] World '{stem}' matched recording "
                  f"{os.path.basename(loose[0])}")
            return _with_imu(loose[0])
        if len(mp4s) == 1:
            print(f"[preflight] World '{stem}': no name match; using the only "
                  f"recording present ({os.path.basename(mp4s[0])})")
            return _with_imu(mp4s[0])
        print(f"[preflight] World '{stem}': no matching recording among "
              f"{len(mp4s)} candidate(s) — skipping flyover analysis "
              f"(mission will run on lidar + camera only)")
        return None, None

    # No world_path (standalone use): only unambiguous when a single recording
    # is present; never guess between several.
    if len(mp4s) == 1:
        return _with_imu(mp4s[0])
    return None, None


def outputs_are_fresh(mp4_path: str) -> bool:
    """
    Return True only when the sim_logs outputs exist AND were produced from
    this exact recording (path stored in .last_recording).
    """
    if not os.path.exists(CSV_OUT) or not os.path.exists(MAP_OUT):
        return False
    if not os.path.exists(_LAST_REC):
        return False
    try:
        with open(_LAST_REC) as f:
            lines = f.read().splitlines()
    except OSError:
        return False
    last      = lines[0].strip() if lines else ""
    last_code = lines[1].strip() if len(lines) > 1 else ""
    if os.path.normcase(os.path.abspath(last)) != os.path.normcase(os.path.abspath(mp4_path)):
        return False
    # Invalidate when this analysis code has changed since the cached run
    if last_code != _code_stamp():
        return False
    # Also require outputs to be newer than the recording itself
    rec_mtime = os.path.getmtime(mp4_path)
    return (os.path.getmtime(CSV_OUT) >= rec_mtime and
            os.path.getmtime(MAP_OUT)  >= rec_mtime)


def _code_stamp() -> str:
    """Identity of this analysis code — cache is invalid when it changes."""
    try:
        st = os.stat(os.path.abspath(__file__))
        return f"{st.st_mtime_ns}:{st.st_size}"
    except OSError:
        return "unknown"


# ══════════════════════════════════════════════════════════════════════════════
#  Coordinate helpers
# ══════════════════════════════════════════════════════════════════════════════

def pix_to_world(px, py, H):
    w = H @ np.array([float(px), float(py), 1.0])
    return w[0] / w[2], w[1] / w[2]


def make_map_to_src_H(H_pix_world):
    """
    Homography: map pixel → source-frame pixel, so that
    cv2.warpPerspective(src, H, (MAP_PX, MAP_PX)) gives a world-aligned top-down tile.
    """
    M_map_world = np.array([
        [ MAP_SCALE,  0.0,       -HALF_M],
        [ 0.0,       -MAP_SCALE,  HALF_M],
        [ 0.0,        0.0,        1.0   ]], dtype=np.float64)
    return np.linalg.inv(H_pix_world) @ M_map_world


def make_grid_to_src_H(H_pix_world):
    """Homography: intermediate-grid pixel → source-frame pixel (60 m grid)."""
    M_grid_world = np.array([
        [ MAP_SCALE,  0.0,       -GRID_HALF_M],
        [ 0.0,       -MAP_SCALE,  GRID_HALF_M],
        [ 0.0,        0.0,        1.0        ]], dtype=np.float64)
    return np.linalg.inv(H_pix_world) @ M_grid_world


def _decompose_similarity(W):
    """(theta, scale, tx, ty) from a near-similarity 3x3 (projective residue dropped)."""
    W = W / W[2, 2]
    s  = math.hypot(W[0, 0], W[1, 0])
    th = math.atan2(W[1, 0], W[0, 0])
    return th, max(s, 1e-9), float(W[0, 2]), float(W[1, 2])


def _partial_similarity(th, s, tx, ty, lam):
    """Fractional similarity: identity at lam=0, full (th,s,t) at lam=1."""
    thl, sl = th * lam, s ** lam
    c, sn = math.cos(thl), math.sin(thl)
    return np.array([
        [sl * c, -sl * sn, tx * lam],
        [sl * sn,  sl * c, ty * lam],
        [0.0,      0.0,    1.0     ]], dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════════════
#  ArUco calibration
# ══════════════════════════════════════════════════════════════════════════════

def detect_aruco(frame) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    d    = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    det  = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    result = {}
    if ids is not None:
        for i, mid in enumerate(ids.flatten()):
            result[int(mid)] = corners[i][0]
    return result


def _orientation_sign(H, p) -> float:
    """Sign of the Jacobian determinant of homography H at image point p.
    Negative = orientation-reversing (expected for pixel→world of a nadir cam)."""
    def proj(q):
        v = H @ np.array([float(q[0]), float(q[1]), 1.0])
        return v[:2] / v[2]
    o  = proj(p)
    ex = proj((p[0] + 5.0, p[1])) - o
    ey = proj((p[0], p[1] + 5.0)) - o
    return float(np.sign(ex[0] * ey[1] - ex[1] * ey[0]))


# The 8 dihedral orderings of a square's corners (4 rotations x optional mirror).
# Needed because pixel→world for a nadir camera is orientation-REVERSING (image
# y points down, world y up): a similarity fit (rotation+scale only) can never
# represent it and silently rotates the whole world frame instead — that bug
# rotated every map/victim estimate produced before v5.
_DIHEDRAL_PERMS = [
    (0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2),
    (0, 3, 2, 1), (3, 2, 1, 0), (2, 1, 0, 3), (1, 0, 3, 2),
]


# Exact mat geometry, decoded from protos/textures/OriginPattern.png rendered
# with the standard Webots Plane texture mapping (verified: all three markers
# decode unmirrored in the flyover video, so the mapping is not flipped):
#   marker 1 centre at (+0.375, 0)  — world +X (east)  of marker 0
#   marker 2 centre at (0, +0.375)  — world +Y (north) of marker 0
# NOTE: the competition README states the opposite ("+X toward Marker 2, +Y
# toward Marker 1") — the RENDERED mat is authoritative, because the marking
# supervisor converts our CSV to world coordinates by pure translation.
MARKER_WORLD_CENTERS = {
    0: np.array([0.0, 0.0]),
    1: np.array([0.375, 0.0]),
    2: np.array([0.0, 0.375]),
}


def build_calibration_H(markers) -> "np.ndarray | None":
    """
    Pixel→world homography from detected ArUco markers.

    All markers on the mat are printed in the SAME orientation, so one corner
    →world ordering (out of 8 dihedral options) applies to every marker.  The
    ordering is disambiguated by (a) requiring the map to be orientation-
    reversing (image y is down, world y is up — mandatory for a downward
    camera) and (b) projecting the other markers onto their exact known mat
    positions.  All corners of all detected known markers are then fit
    together (every world coordinate is exact mat geometry).
    """
    known = {mid: c for mid, c in markers.items() if mid in MARKER_WORLD_CENTERS}
    if 0 not in known or len(known) < 2:
        return None    # marker-0-only frames are handled via build_marker0_H

    c0      = known[0]
    center0 = c0.mean(axis=0)
    side_px = np.mean([np.linalg.norm(c0[(i+1)%4] - c0[i]) for i in range(4)])
    if side_px < 10:
        return None

    others = [(mid, MARKER_WORLD_CENTERS[mid]) for mid in known if mid != 0]

    best = None
    for perm in _DIHEDRAL_PERMS:
        dst0 = np.float32([_M0_IDEAL[perm[i]] for i in range(4)])
        H0, _ = cv2.findHomography(np.float32(c0), dst0)
        if H0 is None or abs(np.linalg.det(H0)) < 1e-12:
            continue
        if _orientation_sign(H0, center0) >= 0:
            continue                          # wrong handedness
        score = 0.0
        for mid, wc in others:
            c = known[mid].mean(axis=0)
            p = H0 @ np.array([float(c[0]), float(c[1]), 1.0])
            score += float(np.linalg.norm(p[:2] / p[2] - wc))
        if best is None or score < best[0]:
            best = (score, perm)
    if best is None:
        return None
    _, perm = best

    src, dst = [], []
    for mid, corners_img in known.items():
        wc = MARKER_WORLD_CENTERS[mid]
        for i in range(4):
            src.append(corners_img[i])
            dst.append(wc + _M0_IDEAL[perm[i]])

    # dst is in METRES — LMedS needs no reprojection threshold (a metric RANSAC
    # threshold is hard to pick and, set wrong, either rejects nothing or
    # strips the fit down to a degenerate 4-point extrapolation).
    H, _ = cv2.findHomography(np.float32(src), np.float32(dst), cv2.LMEDS)
    return H


# Marker 0 corner world coordinates (its centre IS the world origin).  Order is
# unknown a priori (depends on mat print orientation); resolved once from a
# multi-marker calibration frame, then reused for marker-0-only anchor frames.
_M0_HALF = MARKER_SIZE_M / 2.0
_M0_IDEAL = np.array([
    [-_M0_HALF,  _M0_HALF],
    [ _M0_HALF,  _M0_HALF],
    [ _M0_HALF, -_M0_HALF],
    [-_M0_HALF, -_M0_HALF]], dtype=np.float64)


def _H_is_sane(H, frame_w, frame_h) -> bool:
    """Reject degenerate pixel→world homographies (vanishing line crossing the
    frame, absurd scale, wrong handedness)."""
    if H is None or abs(np.linalg.det(H)) < 1e-12:
        return False
    corners = [(0, 0), (frame_w, 0), (frame_w, frame_h), (0, frame_h)]
    pts = []
    for x, y in corners:
        v = H @ np.array([float(x), float(y), 1.0])
        if abs(v[2]) < 1e-9:
            return False
        p = v[:2] / v[2]
        if not np.all(np.isfinite(p)) or np.max(np.abs(p)) > 200.0:
            return False
        pts.append(p)
    w_span = max(np.linalg.norm(pts[1] - pts[0]), np.linalg.norm(pts[2] - pts[3]))
    h_span = max(np.linalg.norm(pts[3] - pts[0]), np.linalg.norm(pts[2] - pts[1]))
    if not (0.25 <= w_span <= 40.0 and 0.25 <= h_span <= 40.0):
        return False
    return _orientation_sign(H, (frame_w / 2.0, frame_h / 2.0)) < 0


def resolve_marker0_permutation(corners0, H_ref):
    """Map detected marker-0 corner order → _M0_IDEAL indices using a trusted H."""
    perm = []
    for c in corners0:
        p = H_ref @ np.array([float(c[0]), float(c[1]), 1.0])
        w = p[:2] / p[2]
        perm.append(int(np.argmin(np.linalg.norm(_M0_IDEAL - w, axis=1))))
    return tuple(perm) if len(set(perm)) == 4 else None


def build_marker0_H(corners0, perm):
    """Pixel→world homography from marker 0's four corners alone."""
    dst = np.float32([_M0_IDEAL[perm[i]] for i in range(4)])
    H, _ = cv2.findHomography(np.float32(corners0), dst)
    return H


def fallback_H(frame):
    """Rough pixel→world for when no ArUco markers are ever visible."""
    h, w = frame.shape[:2]
    s    = 5.0 / (w / 2.0)
    return np.array([[s, 0, -w/2*s],
                     [0, -s,  h/2*s],
                     [0,  0,  1    ]], dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════════════
#  IMU dead reckoning
# ══════════════════════════════════════════════════════════════════════════════

def _integrate_imu_trajectory(imu_path, fps, total, calib_fi):
    """
    Dead-reckon drone (x, y, altitude, heading) from IMU CSV.

    Integrates from t=0 with zero initial velocity.  Horizontally anchors
    the trajectory so that at calib_fi the drone is at world (0, 0) — matching
    the ArUco calibration where marker 0 is the world origin.

    Heading convention: theta = atan2(comp_x, comp_y) gives the angle of the
    drone's body +X axis from world east (+X), counterclockwise.
    Derivation: at theta=0 drone faces east, compass north (world +Y) is at
    body +Y so comp=(0,1,0); at theta=pi/2 drone faces north, comp=(1,0,0).

    Returns dict {frame_idx: (x_m, y_m, alt_m, theta_rad)} or {} on failure.
    """
    if imu_path is None or not os.path.exists(imu_path):
        return {}

    rows = []
    try:
        with open(imu_path, newline='') as fh:
            reader = _csv_mod.DictReader(fh)
            for row in reader:
                try:
                    rows.append({k: float(v) for k, v in row.items()})
                except (ValueError, KeyError):
                    pass
    except Exception:
        return {}

    if len(rows) < 2:
        return {}

    g  = 9.81
    x = y = h = vx = vy = vh = 0.0
    traj = {}

    for i in range(1, len(rows)):
        r0, r1 = rows[i - 1], rows[i]
        dt = r1['timestamp'] - r0['timestamp']
        if dt <= 0 or dt > 1.0:
            continue

        ax  = (r0['acc_x']  + r1['acc_x'])  / 2
        ay  = (r0['acc_y']  + r1['acc_y'])  / 2
        az  = (r0['acc_z']  + r1['acc_z'])  / 2
        cxc = (r0['comp_x'] + r1['comp_x']) / 2
        cyc = (r0['comp_y'] + r1['comp_y']) / 2

        theta    = math.atan2(cxc, cyc)
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        # Rotate body horizontal acceleration to world frame
        a_wx = ax * cos_t - ay * sin_t
        a_wy = ax * sin_t + ay * cos_t
        a_wz = az - g

        vx += a_wx * dt;  vy += a_wy * dt;  vh += a_wz * dt
        x  += vx  * dt;  y  += vy  * dt;  h  += vh  * dt
        h   = max(0.05, h)

        fi = int(round(r1['timestamp'] * fps))
        if 0 <= fi < total:
            traj[fi] = (x, y, h, theta)

    if not traj:
        return {}

    # Anchor horizontal to world (0, 0) at calibration frame
    nearest = min(traj.keys(), key=lambda k: abs(k - calib_fi))
    xa, ya, _, _ = traj[nearest]
    return {fi: (xi - xa, yi - ya, hi, ti)
            for fi, (xi, yi, hi, ti) in traj.items()}


def _pose_to_H(drone_x, drone_y, drone_h, drone_theta,
               theta_0, h0, H_calib, cx, cy):
    """
    Pixel-to-world homography for a drone at (drone_x, drone_y, drone_h) with
    heading drone_theta, given a calibration homography H_calib at calibration
    pose (0, 0, h0, theta_0).

    Formula:  H_frame = T_world @ H_calib @ H_rot(Δθ) @ H_scale(s)

    where:
      T_world   — world-space translation by (drone_x, drone_y)
      H_rot(Δθ) — CCW image rotation by Δθ about principal point (cx, cy),
                  undoing the CW scene rotation caused by drone yaw Δθ
      H_scale(s)— image scale by s = drone_h/h0 about (cx, cy),
                  expanding compressed pixels back to calibration-altitude scale

    Derivation check:
      At calibration pose (Δθ=0, s=1, xd=0, yd=0) → H_frame = H_calib  ✓
      For pure translation → pixel (cx,cy) maps to (xd, yd)              ✓
      For rotation Δθ=π/2 → image-right maps to world-south direction     ✓
    """
    s       = drone_h / max(h0, 0.01)
    d_theta = drone_theta - theta_0
    cos_t, sin_t = math.cos(d_theta), math.sin(d_theta)

    # Image rotation: rotate pixels CCW by Δθ about principal point
    H_rot = np.array([
        [cos_t, -sin_t, cx * (1.0 - cos_t) + cy * sin_t],
        [sin_t,  cos_t, cy * (1.0 - cos_t) - cx * sin_t],
        [0.0,    0.0,   1.0                              ]
    ], dtype=np.float64)

    # Image scale: expand pixel offsets from principal point by s
    H_scale = np.array([
        [s,   0.0, cx * (1.0 - s)],
        [0.0, s,   cy * (1.0 - s)],
        [0.0, 0.0, 1.0            ]
    ], dtype=np.float64)

    # World translation
    T_world = np.array([
        [1.0, 0.0, drone_x],
        [0.0, 1.0, drone_y],
        [0.0, 0.0, 1.0    ]
    ], dtype=np.float64)

    return T_world @ H_calib @ H_rot @ H_scale


def _scan_for_best_calibration(video_path: str, fps: float, total: int):
    """
    Scan the FULL video (sampling every 2 s) and return the frame with the
    most ArUco markers visible.  Returns (H, frame_index, gray_image) or None.
    """
    cap   = cv2.VideoCapture(video_path)
    step  = max(1, int(fps * 2))
    best  = None   # (score, H, fi, gray)

    for fi in range(0, total, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ret, frame = cap.read()
        if not ret:
            break
        markers = detect_aruco(frame)
        if len(markers) < 2:
            continue
        H = build_calibration_H(markers)
        if H is None:
            continue
        score = len(markers) * 10 + (1 if 1 in markers else 0) + (1 if 2 in markers else 0)
        if best is None or score > best[0]:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            best = (score, H, fi, gray)
        if score >= 30:  # all 3 markers, can't do better
            break

    cap.release()
    if best:
        print(f"[calib] Best calibration: frame {best[2]}, {best[0]//10} markers")
        return best[1], best[2], best[3]   # H, fi, gray
    return None


# ══════════════════════════════════════════════════════════════════════════════
#  Vision pose tracking (v5)
#
#  Replaces IMU double-integration (position drifts quadratically) with a
#  frame-to-frame visual odometry chain:
#    - every 1/TRACK_HZ s: ORB features + RANSAC similarity transform to the
#      previous sample (captures translation, yaw AND altitude change)
#    - absolute re-anchoring whenever ArUco markers are visible (multi-marker
#      homography, or marker-0-only via its 4 corners once the corner→world
#      order is resolved)
#    - drift between consecutive anchors is distributed linearly back over the
#      chained samples (classic loop-closure correction)
# ══════════════════════════════════════════════════════════════════════════════

def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _load_compass_yaw(imu_path):
    """
    Per-frame absolute yaw (rad) from the IMU compass columns.  IMU rows are
    written once per video frame, so row index == frame index.  Verified
    empirically on small_world: visual step rotation matches
    atan2(comp_x, comp_y) deltas to ~0.03 deg on well-tracked steps.
    Returns a numpy array, or None when unavailable.
    """
    if imu_path is None or not os.path.exists(imu_path):
        return None
    try:
        yaws = []
        with open(imu_path, newline='') as fh:
            for row in _csv_mod.DictReader(fh):
                yaws.append(math.atan2(float(row["comp_x"]),
                                       float(row["comp_y"])))
        return np.array(yaws) if yaws else None
    except (KeyError, ValueError, OSError):
        return None


def _fit_scale_shift(pr, q):
    """Least-squares s, t for q ≈ s*pr + t (rotation already applied to pr)."""
    pm, qm = pr.mean(axis=0), q.mean(axis=0)
    p0, q0 = pr - pm, q - qm
    denom = float((p0 * p0).sum())
    if denom < 1e-9:
        return None, None
    s = float((p0 * q0).sum() / denom)
    return s, qm - s * pm


def _step_with_compass(sp, dp, dpsi, centre=None,
                       s_lo=None, s_hi=None):
    """
    RANSAC for q = s*R(dpsi)*p + t with the rotation FIXED by the compass.
    Sampling 2-point hypotheses; a hypothesis is only valid when the point
    displacement is parallel to the rotated baseline (which is exactly the
    compass-consistency test).

    Among well-supported models, the one whose image-centre motion is
    SMALLEST is preferred: every static plane above the floor (furniture and
    wall tops) is closer to the camera and so moves faster in the image, and
    in-plane rotation moves the principal point not at all — so the
    slowest-moving strong consensus is the floor, the plane the ground
    homography must describe.  Picking max-consensus instead occasionally
    locks onto a large table/bed and inflates the step by its height ratio,
    a bias that is always positive and compounds over hundreds of steps.

    Returns (3x3 similarity, inlier count) or (None, 0).
    """
    if s_lo is None:
        s_lo = STEP_SCALE_MIN
    if s_hi is None:
        s_hi = STEP_SCALE_MAX
    c, s_ = math.cos(dpsi), math.sin(dpsi)
    R = np.array([[c, -s_], [s_, c]])
    pr = sp @ R.T
    n = len(pr)
    if centre is None:
        centre = (sp.max(axis=0) + sp.min(axis=0)) / 2.0
    ctr_r = R @ np.asarray(centre, dtype=np.float64)
    rng = np.random.default_rng(12345)
    cands, best_cnt = [], 0
    for _ in range(200):
        i, j = rng.integers(0, n, 2)
        if i == j:
            continue
        d = pr[j] - pr[i]
        e = dp[j] - dp[i]
        L2 = float(d @ d)
        if L2 < 400.0:                      # baseline >= 20 px
            continue
        s = float(d @ e) / L2
        if not (s_lo <= s <= s_hi):
            continue
        # displacement must be parallel to the rotated baseline
        if float(np.linalg.norm(e - s * d)) > 0.06 * math.sqrt(L2):
            continue
        t = dp[i] - s * pr[i]
        resid = np.linalg.norm(dp - (s * pr + t), axis=1)
        inl = resid < 3.0
        cnt = int(inl.sum())
        if cnt >= MIN_STEP_INLIERS:
            dc = float(np.linalg.norm(s * ctr_r + t - centre))
            cands.append((cnt, dc, inl))
        best_cnt = max(best_cnt, cnt)
    if best_cnt < MIN_STEP_INLIERS:
        return None, 0
    strong = [cd for cd in cands if cd[0] >= max(MIN_STEP_INLIERS,
                                                 int(0.5 * best_cnt))]
    cnt, _, inl = min(strong, key=lambda cd: cd[1])
    s, t = _fit_scale_shift(pr[inl], dp[inl])
    if s is None or not (s_lo <= s <= s_hi):
        return None, 0
    return np.array([[s * c, -s * s_, t[0]],
                     [s * s_,  s * c,  t[1]],
                     [0.0,     0.0,    1.0]]), cnt


def _estimate_step_full(prev_feat, cur_feat, bf, dpsi=None, centre=None,
                        s_lo=None, s_hi=None):
    """
    Similarity 3x3 mapping prev-sample pixels → current-sample pixels, plus
    the supporting inlier count:  returns (M, n_inliers) or (None, 0).
    When dpsi (compass yaw change, rad) is given, the rotation is constrained
    to it — the compass is absolute and drift-free, while unconstrained RANSAC
    picks up a fraction-of-a-degree rotation bias per step whenever matched
    features sit on planes of different heights (wall tops vs floor), which
    compounds to tens of degrees over the flight.
    """
    kp1, d1 = prev_feat
    kp2, d2 = cur_feat
    if d1 is None or d2 is None or len(d1) < 20 or len(d2) < 20:
        return None, 0
    matches = bf.match(d1, d2)
    if len(matches) < MIN_STEP_INLIERS:
        return None, 0
    matches = sorted(matches, key=lambda m: m.distance)[:300]
    sp = np.float32([kp1[m.queryIdx].pt for m in matches])
    dp = np.float32([kp2[m.trainIdx].pt for m in matches])
    if dpsi is not None:
        return _step_with_compass(sp, dp, dpsi, centre, s_lo, s_hi)
    M, inliers = cv2.estimateAffinePartial2D(
        sp, dp, method=cv2.RANSAC, ransacReprojThreshold=3.0, maxIters=3000)
    ninl = 0 if inliers is None else int(inliers.sum())
    if M is None or ninl < MIN_STEP_INLIERS:
        return None, 0
    s = math.hypot(M[0, 0], M[1, 0])
    if not (STEP_SCALE_MIN <= s <= STEP_SCALE_MAX):
        return None, 0
    return np.vstack([M, [0.0, 0.0, 1.0]]), ninl


def _estimate_step(prev_feat, cur_feat, bf, dpsi=None, centre=None):
    return _estimate_step_full(prev_feat, cur_feat, bf, dpsi, centre)[0]


def _dpsi(yaw, fa, fb):
    """Compass yaw change between two frame indices (None if no compass)."""
    if yaw is None:
        return None
    a = min(fa, len(yaw) - 1)
    b = min(fb, len(yaw) - 1)
    return _wrap_angle(float(yaw[b] - yaw[a]))


def _fine_chain(video_path, fi_a, fi_b, feat_a, feat_b, orb, bf,
                do_undist=False, K=None, dist=None, yaw=None, centre=None):
    """
    Recover the fi_a→fi_b step by chaining sub-steps through intermediate
    frames.  The 4 Hz sampling can have near-zero overlap when the drone flies
    low (ground displacement per sample grows as altitude shrinks); stepping
    through the full-rate frames always overlaps.  Bisects recursively down to
    single-frame steps, reading intermediates from a secondary capture.
    """
    cap = cv2.VideoCapture(video_path)
    feats = {fi_a: feat_a, fi_b: feat_b}

    def feat_at(fi):
        if fi not in feats:
            cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
            ok, frame = cap.read()
            if not ok:
                feats[fi] = None
            else:
                if do_undist:
                    frame = cv2.undistort(frame, K, dist)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                feats[fi] = orb.detectAndCompute(gray, None)
        return feats[fi]

    def chain(a, b, direct_ok=True):
        if direct_ok:
            fa, fb = feat_at(a), feat_at(b)
            if fa is None or fb is None:
                return None
            M = _estimate_step(fa, fb, bf, _dpsi(yaw, a, b), centre)
            if M is not None:
                return M
        if b - a < 2:
            return None
        mid = (a + b) // 2
        M1 = chain(a, mid)
        if M1 is None:
            return None
        M2 = chain(mid, b)
        return None if M2 is None else M2 @ M1

    M = chain(fi_a, fi_b, direct_ok=False)   # direct match already failed
    cap.release()
    return M


def _pose_similarity_parts(H):
    """
    Approximate (theta, log_scale, t) of a pixel→world pose homography.
    Poses are orientation-REVERSING (image y down, world y up), so the linear
    part is modelled as A = s·R(theta)·F with F = diag(1,-1); A·F undoes the
    flip and leaves the pure rotation.
    """
    A = H[:2, :2] / H[2, 2]
    B = A @ np.diag([1.0, -1.0])
    th = math.atan2(B[1, 0] - B[0, 1], B[0, 0] + B[1, 1])
    s = math.sqrt(max(abs(float(np.linalg.det(A))), 1e-12))
    t = H[:2, 2] / H[2, 2]
    return th, math.log(s), t


def _global_solve(recs, yaw):
    """
    Global least-squares over all per-sample similarity poses.

    With every edge rotation pinned by the compass, pose rotations are known
    in closed form (theta_i + psi_i is constant, fixed by the ArUco anchors),
    and the remaining unknowns decouple into two LINEAR problems solved over
    the whole graph at once:
        scales:        lam_j - lam_i = ln s_edge        (lam = ln scale)
        translations:  t_j  - t_i    = s_i R_i t_edge
    for each edge M mapping j-sample pixels → i-sample pixels.  This replaces
    the sequential drift-distribution heuristic (which hard-pins each loop
    closure to an already-drifted earlier strip) with a solution that balances
    every chain step, every loop closure and every ArUco anchor globally.
    A second round drops edges whose residuals mark them as mismatches.

    Returns a pose list (None for samples not connected to any anchor), or
    None when the solve is not applicable.
    """
    n = len(recs)
    edges = []                                     # (j, i, M, weight)
    for i, r in enumerate(recs):
        if r["step"] is not None and r["step_from"] is not None:
            edges.append((r["step_from"], i, r["step"], 1.0))
        for j2, M2 in r.get("lc_edges", []):
            edges.append((j2, i, M2, 2.0))
    # conf: 2.0 for multi-marker anchors (build_calibration_H, fit across up
    # to 3 markers' corners), 1.0 for weaker single-marker anchors
    # (build_marker0_H) — see track_poses where anchor_conf is tagged.
    anchors = [(i, r["H_abs"], r.get("anchor_conf", 1.0)) for i, r in enumerate(recs)
               if r["H_abs"] is not None]
    if not anchors or yaw is None or not edges:
        return None

    # restrict to the anchor-connected component (anything else is unposable)
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for j, i, _, _ in edges:
        parent[find(j)] = find(i)
    roots = {find(a) for a, _, _ in anchors}
    keep = [i for i in range(n) if find(i) in roots]
    idx = {i: k for k, i in enumerate(keep)}
    m = len(keep)
    edges = [e for e in edges if e[0] in idx and e[1] in idx]

    # With A_i = s_i R(theta_i) F and step rotations equal to the compass
    # delta-psi, chaining gives theta_b = theta_a + dpsi_ab, i.e.
    # theta_i - psi_i is one global constant, fixed by the anchors.
    psi = np.array([_dpsi(yaw, 0, r["fi"]) for r in recs])
    cs = [_pose_similarity_parts(H)[0] - psi[i] for i, H, _ in anchors]
    c0 = math.atan2(float(np.mean(np.sin(cs))), float(np.mean(np.cos(cs))))
    theta = np.array([_wrap_angle(c0 + p) for p in psi])

    W_ANCH = 45.0   # was 30.0 — long flights accumulate hundreds of chain
                    # edges vs a handful of anchors; raise the base pin
                    # strength so anchors dominate more of the LS solve
    lam = T = None
    for rnd in range(2):
        rows, rhs_l = [], []
        for j, i, M, w in edges:
            sm = math.hypot(M[0, 0], M[1, 0])
            row = np.zeros(m)
            row[idx[j]] = w
            row[idx[i]] = -w
            rows.append(row)
            rhs_l.append(w * math.log(sm))
        for a, H, conf in anchors:
            row = np.zeros(m)
            row[idx[a]] = W_ANCH * conf
            rows.append(row)
            rhs_l.append(W_ANCH * conf * _pose_similarity_parts(H)[1])
        lam, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs_l), rcond=None)

        def edge_offset(j, i, M):
            # t_j - t_i = s_i R(theta_i) F t_M   (F flips the y component)
            s_i = math.exp(lam[idx[i]])
            ci, si_ = math.cos(theta[i]), math.sin(theta[i])
            tx, ty = float(M[0, 2]), -float(M[1, 2])
            return np.array([s_i * (ci * tx - si_ * ty),
                             s_i * (si_ * tx + ci * ty)])

        rows, rhs_t = [], []
        for j, i, M, w in edges:
            row = np.zeros(m)
            row[idx[j]] = w
            row[idx[i]] = -w
            rows.append(row)
            rhs_t.append(w * edge_offset(j, i, M))
        for a, H, conf in anchors:
            row = np.zeros(m)
            row[idx[a]] = W_ANCH * conf
            rows.append(row)
            rhs_t.append(W_ANCH * conf * _pose_similarity_parts(H)[2])
        T, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs_t), rcond=None)

        if rnd == 0:
            good = []
            for j, i, M, w in edges:
                sm = math.hypot(M[0, 0], M[1, 0])
                lres = abs(lam[idx[j]] - lam[idx[i]] - math.log(sm))
                tres = float(np.linalg.norm(
                    T[idx[j]] - T[idx[i]] - edge_offset(j, i, M)))
                if lres <= 0.15 and tres <= 0.75:
                    good.append((j, i, M, w))
            if len(good) == len(edges):
                break
            # keep the graph over-determined; bail out of rejection otherwise
            if len(good) < m:
                break
            edges = good

    poses = [None] * n
    for i in keep:
        s = math.exp(lam[idx[i]])
        c, s_ = math.cos(theta[i]), math.sin(theta[i])
        # A = s R(theta) F  (orientation-reversing, as required for poses)
        poses[i] = np.array([[s * c,  s * s_, T[idx[i]][0]],
                             [s * s_, -s * c, T[idx[i]][1]],
                             [0.0,     0.0,   1.0]])

    # Anchor frames have a genuine projective homography (from ArUco corner
    # geometry, not just a similarity fit) — restore it exactly rather than
    # keeping the similarity-only reconstruction above, which discards
    # perspective/shear the solve above never modelled in the first place.
    # Every anchor index is its own connected-component root, so it's always
    # in `keep`.
    for a, H, _ in anchors:
        poses[a] = H
    return poses


def track_poses(video_path: str, fps: float, total: int, K=None, dist=None,
                yaw=None):
    """
    PASS 1 over the video: build a pixel→world homography for every tracking
    sample.  Returns (poses, n_samples) where poses is a list of
    (frame_idx, H_pix_world) for each successfully-tracked sample.
    """
    step_every = max(1, int(round(fps / TRACK_HZ)))
    do_undist  = (K is not None and dist is not None and
                  np.any(np.abs(dist) > 1e-6))

    orb = cv2.ORB_create(nfeatures=3000, fastThreshold=12)
    bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Each new sample is matched against the last "target" sample (the most
    # recent one that produced a step or an absolute anchor).  A failed step
    # therefore doesn't kill the chain — following samples bridge over it,
    # up to MAX_BRIDGE samples.
    recs = []
    tgt_feat, tgt_i = None, None
    cap = cv2.VideoCapture(video_path)
    fi  = -1
    while True:
        if not cap.grab():
            break
        fi += 1
        if fi % step_every:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        if do_undist:
            frame = cv2.undistort(frame, K, dist)
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        f_h, f_w = gray.shape[:2]
        markers = detect_aruco(frame)
        anchor_H = build_calibration_H(markers) if len(markers) >= 2 else None
        if anchor_H is not None and not _H_is_sane(anchor_H, f_w, f_h):
            anchor_H = None
        m0 = markers.get(0)
        feat = orb.detectAndCompute(gray, None)

        i    = len(recs)
        step = None
        if tgt_feat is not None and (i - tgt_i) <= MAX_BRIDGE:
            tgt_fi = recs[tgt_i]["fi"]
            f_ctr  = (f_w / 2.0, f_h / 2.0)
            step = _estimate_step(tgt_feat, feat, bf, _dpsi(yaw, tgt_fi, fi),
                                  f_ctr)
            if step is None and tgt_i == i - 1:
                # Consecutive samples that fail to match usually mean the
                # drone is low and fast — recover through full-rate frames.
                step = _fine_chain(video_path, tgt_fi, fi,
                                   tgt_feat, feat, orb, bf, do_undist, K, dist,
                                   yaw, f_ctr)
                if step is not None:
                    print(f"[track] fine-chained step {tgt_fi}->{fi}")
        recs.append({"fi": fi, "step": step,
                     "step_from": tgt_i if step is not None else None,
                     "anchor_H": anchor_H, "m0": m0, "feat": feat})
        if step is not None or anchor_H is not None or m0 is not None or tgt_feat is None:
            tgt_feat, tgt_i = feat, i
    cap.release()

    if not recs:
        return [], 0

    # Resolve marker-0 corner ordering from the first multi-marker frame that
    # also sees marker 0, enabling marker-0-only absolute anchors.
    perm = None
    for r in recs:
        if r["anchor_H"] is not None and r["m0"] is not None:
            perm = resolve_marker0_permutation(r["m0"], r["anchor_H"])
            if perm is not None:
                break

    n_anchors = 0
    for r in recs:
        H_abs = r["anchor_H"]
        multi_marker = H_abs is not None
        if H_abs is None and perm is not None and r["m0"] is not None:
            c0 = r["m0"]
            side_px = np.mean([np.linalg.norm(c0[(i + 1) % 4] - c0[i]) for i in range(4)])
            if side_px >= 25:                      # corners too noisy when tiny
                H_abs = build_marker0_H(c0, perm)
        if H_abs is not None and not _H_is_sane(H_abs, f_w, f_h):
            H_abs = None
        r["H_abs"] = H_abs
        # Multi-marker anchors (build_calibration_H, fit across up to 3
        # markers' corners) are more reliable than single-marker ones
        # (build_marker0_H) — weight them higher in _global_solve.
        r["anchor_conf"] = 2.0 if multi_marker else 1.0
        n_anchors += H_abs is not None

    # Chain solve with drift distribution between anchors.  Poses propagate
    # along step edges (i inherits from step_from); anchors (ArUco absolute,
    # or loop-closure pseudo-anchors) override and the chain-vs-anchor
    # discrepancy is spread back over the preceding segment.
    n = len(recs)

    def solve():
        poses = [None] * n
        seg   = []                                 # chain-posed indices since last anchor
        for i, r in enumerate(recs):
            chained = None
            j = r["step_from"]
            if j is not None and poses[j] is not None:
                chained = poses[j] @ np.linalg.inv(r["step"])
            H_a = r["H_abs"] if r["H_abs"] is not None else r.get("H_lc")
            if H_a is not None:
                if chained is not None and seg:
                    W = H_a @ np.linalg.inv(chained)  # world→world drift here
                    th, s, tx, ty = _decompose_similarity(W)
                    L = len(seg) + 1
                    for k, jj in enumerate(seg, start=1):
                        poses[jj] = _partial_similarity(th, s, tx, ty, k / L) @ poses[jj]
                poses[i] = H_a
                seg = []
            elif chained is not None:
                poses[i] = chained
                seg.append(i)
        return poses

    poses = solve()

    # Loop closures: the flyover is a serpentine sweep plus low victim-
    # inspection dives.  Adjacent strips revisit the same rooms seconds apart
    # while only the first strips carry ArUco anchors, so re-matching each
    # sample against sufficiently-old samples turns strip overlap into
    # pseudo-anchors, which the solver above treats exactly like ArUco fixes —
    # bounding drift to roughly one strip instead of the whole path.
    #
    # Dive samples (footprint span far from the cruise median) are where the
    # ground-plane odometry breaks — and where victims are filmed close-up —
    # so they get a wider search, cross-altitude scale bounds (ORB matches
    # reliably across a 2-4x zoom) and a brute-force fallback.  A dive frame
    # re-anchored to correctly-posed cruise imagery projects every floor
    # pixel (victims included) correctly, however oblique the view.
    frame_c = np.array([f_w / 2.0, f_h / 2.0, 1.0])
    f_ctr   = (f_w / 2.0, f_h / 2.0)

    def span_of(H):
        a = H @ np.array([0.0, f_h / 2.0, 1.0])
        b = H @ np.array([float(f_w), f_h / 2.0, 1.0])
        return float(np.linalg.norm(b[:2] / b[2] - a[:2] / a[2]))

    for it in range(LC_ITERS):
        centres, spans = {}, {}
        for i in range(n):
            if poses[i] is not None and _H_is_sane(poses[i], f_w, f_h):
                w = poses[i] @ frame_c
                centres[i] = w[:2] / w[2]
                spans[i]   = span_of(poses[i])
        if not spans:
            break
        med_span = float(np.median(list(spans.values())))
        cruise = [i for i in centres
                  if 0.75 * med_span <= spans[i] <= 1.35 * med_span]
        n_lc = n_dive = 0
        for i, r in enumerate(recs):
            r["H_lc"] = None
            r["lc_edges"] = []
            if i not in centres or r["H_abs"] is not None:
                continue
            dive = not (0.75 * med_span <= spans[i] <= 1.35 * med_span)
            radius = LC_RADIUS_DIVE if dive else LC_RADIUS_M
            s_lo, s_hi = (0.25, 4.0) if dive else (None, None)
            t_max = 15.0 if dive else 6.0
            s_rng = (0.3, 3.0) if dive else (0.7, 1.4)
            max_edges = 3 if dive else 2
            cand = [(float(np.linalg.norm(centres[i] - centres[j])), j)
                    for j in cruise if j <= i - LC_MIN_GAP]
            cand = sorted(c for c in cand if c[0] < radius)
            cand = [j for _, j in cand[:(6 if dive else 3)]]
            if dive and not cand:
                # pose estimate too far gone for a spatial search — try a
                # thinned sweep of all old cruise keyframes instead
                cand = [j for j in cruise[::5] if j <= i - LC_MIN_GAP]
            for j in cand:
                M, ninl = _estimate_step_full(
                    recs[j]["feat"], r["feat"], bf,
                    _dpsi(yaw, recs[j]["fi"], r["fi"]), f_ctr, s_lo, s_hi)
                if M is None or ninl < LC_MIN_INL:
                    continue
                H_ps = poses[j] @ np.linalg.inv(M)
                if not _H_is_sane(H_ps, f_w, f_h):
                    continue
                # the implied correction must look like drift, not a mismatch
                th, s, tx, ty = _decompose_similarity(
                    H_ps @ np.linalg.inv(poses[i]))
                if abs(th) > math.radians(8.0) or not (s_rng[0] <= s <= s_rng[1]) \
                   or math.hypot(tx, ty) > t_max:
                    continue
                r["lc_edges"].append((j, M))
                if r["H_lc"] is None:          # sequential-solve fallback
                    r["H_lc"] = H_ps
                    n_lc += 1
                    n_dive += dive
                if len(r["lc_edges"]) >= max_edges:
                    break
        if n_lc == 0:
            break
        gposes = _global_solve(recs, yaw)
        poses = gposes if gposes is not None else solve()
        print(f"[track] loop-closure pass {it + 1}: {n_lc} closures "
              f"({n_dive} at dive samples, "
              f"{'global' if gposes is not None else 'sequential'} solve)")

    tracked = [(recs[i]["fi"], poses[i]) for i in range(n)
               if poses[i] is not None and _H_is_sane(poses[i], f_w, f_h)]
    print(f"[track] {len(tracked)}/{n} samples posed  ({n_anchors} absolute anchors, "
          f"marker0 perm={'resolved' if perm else 'n/a'})")
    return tracked, n


def _make_ref_grid_H(H_pix_world):
    """Homography: refinement-grid pixel → source-frame pixel (0.10 m/px)."""
    half = REF_PX * REF_SCALE / 2.0
    M_ref_world = np.array([
        [ REF_SCALE,  0.0,       -half],
        [ 0.0,       -REF_SCALE,  half],
        [ 0.0,        0.0,        1.0 ]], dtype=np.float64)
    return np.linalg.inv(H_pix_world) @ M_ref_world


def _warp_to_ref(gray, mask, H_pix_world):
    """Warp a frame + validity mask into the refinement grid. Returns (img, m)."""
    H_gs = _make_ref_grid_H(H_pix_world)
    wg = cv2.warpPerspective(gray, H_gs, (REF_PX, REF_PX),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    wm = cv2.warpPerspective(mask, H_gs, (REF_PX, REF_PX),
                             flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
    return wg, wm >= 250


def refine_poses_mosaic(video_path, poses, do_undistort=False, K=None, dist=None):
    """
    PASS 1.5: re-register every posed sample against the consensus mean mosaic.

    Builds a coarse cruise-only mean mosaic from the current poses, then for
    each sample finds the world-translation that best aligns its warped view
    with the mosaic (masked NCC over a local search window) and applies it to
    the pose.  Iterates REF_ITERS times.  Corrections are damped and gated on
    match quality; samples that already agree with the consensus are left
    untouched, so the ArUco-anchored strips keep the map absolutely placed.
    """
    if len(poses) < 10:
        return poses
    poses = dict(poses)                      # fi -> H (copy; we mutate)
    last_med = 0.0

    def span_of(H, w_px, h_px):
        a = H @ np.array([0.0, h_px / 2.0, 1.0])
        b = H @ np.array([w_px, h_px / 2.0, 1.0])
        return float(np.linalg.norm(b[:2] / b[2] - a[:2] / a[2]))

    for it in range(REF_ITERS):
        pre_iter = dict(poses)                # state before this iteration's corrections
        # decode all posed frames once per iteration, keep coarse warps
        warped = {}                          # fi -> (img, mask, is_dive)
        spans = {}
        cap = cv2.VideoCapture(video_path)
        fi, mask0 = -1, None
        while True:
            if not cap.grab():
                break
            fi += 1
            if fi not in poses:
                continue
            ok, frame = cap.retrieve()
            if not ok:
                break
            if do_undistort:
                frame = cv2.undistort(frame, K, dist)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h_px, w_px = gray.shape[:2]
            if mask0 is None or mask0.shape != gray.shape:
                mask0 = np.zeros_like(gray)
                mh, mw = int(h_px * CROP_MARGIN_F), int(w_px * CROP_MARGIN_F)
                mask0[mh:h_px - mh, mw:w_px - mw] = 255
            spans[fi] = span_of(poses[fi], w_px, h_px)
            warped[fi] = _warp_to_ref(gray, mask0, poses[fi])
        cap.release()
        if not spans:
            return sorted(poses.items())

        med_span = float(np.median(list(spans.values())))
        is_dive = {fi: not (0.75 * med_span <= s <= 1.35 * med_span)
                   for fi, s in spans.items()}

        # consensus mean mosaic from cruise samples only
        acc = np.zeros((REF_PX, REF_PX), np.float64)
        cnt = np.zeros((REF_PX, REF_PX), np.uint16)
        for fi, (wg, wm) in warped.items():
            if is_dive[fi]:
                continue
            acc[wm] += wg[wm]
            cnt[wm] += 1
        mosaic_ok = cnt > 0
        mosaic = np.where(mosaic_ok, acc / np.maximum(cnt, 1), 0).astype(np.float32)

        n_corr, mags = 0, {}
        dive_seen = dive_corr = cruise_seen = cruise_corr = 0
        dive_maxv, cruise_maxv = [], []
        dive_own_skip = dive_search_skip = 0
        for fi, (wg, wm) in warped.items():
            dive = is_dive[fi]
            search_m = REF_SEARCH_DIVE if dive else REF_SEARCH_M
            s_px = max(2, int(round(search_m / REF_SCALE)))
            ys, xs = np.nonzero(wm)
            if len(xs) < 400:
                if dive:
                    dive_own_skip += 1
                continue
            x0, x1 = xs.min(), xs.max() + 1
            y0, y1 = ys.min(), ys.max() + 1
            templ = wg[y0:y1, x0:x1].astype(np.float32)
            tmask = wm[y0:y1, x0:x1]
            sx0, sx1 = max(x0 - s_px, 0), min(x1 + s_px, REF_PX)
            sy0, sy1 = max(y0 - s_px, 0), min(y1 + s_px, REF_PX)
            search = mosaic[sy0:sy1, sx0:sx1].copy()
            smask = mosaic_ok[sy0:sy1, sx0:sx1]
            if smask.sum() < 400:
                if dive:
                    dive_search_skip += 1
                continue
            if dive:
                dive_seen += 1
            else:
                cruise_seen += 1
            # zero-mean within valid areas so CCORR behaves like NCC
            templ = np.where(tmask, templ - templ[tmask].mean(), 0.0).astype(np.float32)
            search = np.where(smask, search - search[smask].mean(), 0.0).astype(np.float32)
            try:
                resp = cv2.matchTemplate(search, templ, cv2.TM_CCORR_NORMED,
                                         mask=tmask.astype(np.float32))
            except cv2.error:
                continue
            resp = np.nan_to_num(resp, nan=-1.0, posinf=-1.0, neginf=-1.0)
            _, maxv, _, maxloc = cv2.minMaxLoc(resp)
            (dive_maxv if dive else cruise_maxv).append(float(maxv))
            if maxv < REF_MIN_RESP:
                continue
            dx_px = (sx0 + maxloc[0]) - x0
            dy_px = (sy0 + maxloc[1]) - y0
            if dx_px == 0 and dy_px == 0:
                continue
            # grid +x = world +x, grid +y = world -y
            dx_w = REF_DAMP * dx_px * REF_SCALE
            dy_w = -REF_DAMP * dy_px * REF_SCALE
            T = np.array([[1.0, 0.0, dx_w],
                          [0.0, 1.0, dy_w],
                          [0.0, 0.0, 1.0]])
            poses[fi] = T @ poses[fi]
            n_corr += 1
            mags[fi] = math.hypot(dx_w, dy_w)
            if dive:
                dive_corr += 1
            else:
                cruise_corr += 1
        last_med = float(np.median(list(mags.values()))) if mags else 0.0
        print(f"[refine] pass {it + 1}: corrected {n_corr}/{len(warped)} samples "
              f"(median shift {last_med:.2f} m)")
        n_dive_total = dive_seen + dive_own_skip + dive_search_skip
        if n_dive_total:
            dv = np.median(dive_maxv) if dive_maxv else float("nan")
            cv = np.median(cruise_maxv) if cruise_maxv else float("nan")
            print(f"[refine] pass {it + 1}: dive {dive_corr}/{dive_seen} attempted-and-corrected "
                  f"(median NCC peak {dv:.2f}) vs cruise {cruise_corr}/{cruise_seen} "
                  f"(median NCC peak {cv:.2f}) — REF_MIN_RESP={REF_MIN_RESP}")
            print(f"[refine] pass {it + 1}: dive skip breakdown — "
                  f"{dive_own_skip}/{n_dive_total} own-footprint <400px, "
                  f"{dive_search_skip}/{n_dive_total} mosaic-search-region <400px")
        if n_corr == 0:
            break

        # Per-sample convergence gate: a sample whose correction THIS
        # iteration is still large means that specific match was unreliable
        # (sparse coverage, repetitive texture) — revert just that sample to
        # its pre-this-iteration pose (keeping any earlier-iteration
        # correction it already had) rather than discarding the whole pass
        # for every sample, or discarding this sample's earlier good work
        # too. This lets worlds where a subset of (typically dive) samples
        # drag the aggregate median up still keep the benefit everywhere
        # else, without mixing "corrected this pass" and "reverted all the
        # way to pre-refinement" poses into the same mosaic.
        n_reverted = 0
        for fi, mag in mags.items():
            if mag >= 0.25:
                poses[fi] = pre_iter[fi]
                n_reverted += 1
        if n_reverted:
            print(f"[refine] pass {it + 1}: reverted {n_reverted}/{len(mags)} "
                  f"samples with correction >= 0.25 m")

    return sorted(poses.items())


# ══════════════════════════════════════════════════════════════════════════════
#  Victim detection
# ══════════════════════════════════════════════════════════════════════════════

def _estimate_scale_mpp(H_pix_world) -> float:
    """Estimate metres-per-pixel from the homography (average x/y scale)."""
    try:
        # Scale columns of the rotation sub-matrix give m/px
        sx = np.linalg.norm(H_pix_world[:2, 0])
        sy = np.linalg.norm(H_pix_world[:2, 1])
        return (sx + sy) / 2.0
    except Exception:
        return 0.01


# ══════════════════════════════════════════════════════════════════════════════
#  Camera intrinsic calibration
# ══════════════════════════════════════════════════════════════════════════════

def calibrate_intrinsics(video_path: str, fps: float, total: int):
    """
    Estimate (K, dist_coeffs) for the flyover camera.

    Strategy 1 – chessboard (first 30 s, drone hovers above origin mat):
        The origin mat has a 9x6 chessboard (tile = 45 mm).  Four or more
        detected views give calibrateCamera() enough information to solve for
        focal length and radial distortion with the principal point fixed at
        the image centre.

    Strategy 2 – ArUco fallback:
        If fewer than 4 chessboard views are found (common in top-down flybys
        where the pattern barely changes angle), estimate focal length from the
        pixel-per-metre scale of the best ArUco calibration frame, assuming a
        ~5 m flight altitude.  Distortion coefficients are set to zero.

    Returns:
        (K, dist)  –  both numpy arrays, or (None, None) on failure.
    """
    W_px, H_px = None, None

    # ── Strategy 1: chessboard calibration ───────────────────────────────────
    nc, nr  = CHESSBOARD_INNER          # (8, 5)
    sq      = CHESSBOARD_SQ_M
    obj_one = np.zeros((nc * nr, 3), np.float32)
    obj_one[:, :2] = np.indices((nc, nr)).T.reshape(-1, 2) * sq

    all_obj, all_img = [], []
    cap     = cv2.VideoCapture(video_path)
    step    = max(1, int(fps))
    scan_to = min(total, int(fps * 30))

    for fi in range(0, scan_to, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if W_px is None:
            H_px, W_px = gray.shape
        ok, corners = cv2.findChessboardCorners(
            gray, CHESSBOARD_INNER, cv2.CALIB_CB_FAST_CHECK)
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (7, 7), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            all_obj.append(obj_one)
            all_img.append(corners)
    cap.release()

    if W_px is None:
        return None, None

    cx0, cy0 = W_px / 2.0, H_px / 2.0

    if len(all_obj) >= 4:
        K0 = np.array([[max(W_px, H_px), 0,   cx0],
                       [0,   max(W_px, H_px),  cy0],
                       [0,   0,   1             ]], dtype=np.float64)
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS |
                 cv2.CALIB_FIX_PRINCIPAL_POINT  |
                 cv2.CALIB_ZERO_TANGENT_DIST)
        try:
            rms, K, dist, _, _ = cv2.calibrateCamera(
                all_obj, all_img, (W_px, H_px), K0.copy(), None, flags=flags)
            print(f"[intrinsics] Chessboard OK  {len(all_obj)} frames  RMS={rms:.3f}"
                  f"  fx={K[0,0]:.1f}  fy={K[1,1]:.1f}"
                  f"  k1={float(dist.ravel()[0]):.5f}")
            return K, dist
        except cv2.error as e:
            print(f"[intrinsics] calibrateCamera failed: {e}")
    else:
        print(f"[intrinsics] {len(all_obj)} chessboard view(s) in first 30 s"
              f" (need >= 4) — using ArUco fallback")

    # ── Strategy 2: ArUco-based focal-length estimate ─────────────────────────
    # ppm = pixels per metre at the calibration frame.
    # Pinhole geometry: ppm = f / Z  ->  f = ppm * Z.
    # We assume Z ~ 5 m (typical hover altitude) to get an approximate f.
    calib = _scan_for_best_calibration(video_path, fps, total)
    if calib is not None:
        H_cal, _, _ = calib
        mpp = _estimate_scale_mpp(H_cal)
        if mpp > 0:
            Z_assume = 5.0
            f_est    = float(np.clip(Z_assume / mpp, 300.0, 8000.0))
            K    = np.array([[f_est, 0,     cx0],
                             [0,     f_est, cy0],
                             [0,     0,     1  ]], dtype=np.float64)
            dist = np.zeros(5, dtype=np.float64)
            print(f"[intrinsics] ArUco fallback: f={f_est:.1f}  dist=0"
                  f"  (assumed {Z_assume} m altitude)")
            return K, dist

    print("[intrinsics] Calibration failed — no undistortion will be applied")
    return None, None


def _denim_candidates(frame, H_pix_world, mpp, drone_x, drone_y) -> list:
    """
    Denim-component victim detector (v6, primary).

    Every Webots victim model wears blue denim (jeans / skirt / jacket) and,
    crucially, nothing else in the dev worlds is BOTH denim-coloured AND
    limb-sized: wood/floors live at H 8-28, beds with blue covers merge into
    components far wider than a body, and table clutter carries no denim at
    all.  So: threshold the denim hue band, bridge fragments with a ~0.10 m
    dilation (trouser legs + jacket join; a victim stays separate from a bed
    ~0.3 m away), and keep components whose METRIC size fits a human or a
    protruding pair of legs (victims are often half-hidden under furniture).
    """
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    denim = cv2.inRange(hsv, np.array([95, 35, 25]), np.array([135, 255, 255]))
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    denim = cv2.morphologyEx(denim, cv2.MORPH_OPEN, k3)
    # any-clothing mask (non-wood hues + strong reds) — used to pull the
    # centroid from the jeans toward the torso, i.e. the body centre
    cloth = (cv2.inRange(hsv, np.array([30,  50, 25]), np.array([170, 255, 255]))
             | cv2.inRange(hsv, np.array([0, 110, 25]), np.array([6,   255, 255]))
             | cv2.inRange(hsv, np.array([172, 110, 25]), np.array([180, 255, 255])))

    pad_m = 0.10
    kd  = min(151, max(3, int(round(2 * pad_m / mpp)) | 1))
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kd, kd))
    merged = cv2.dilate(denim, ker)

    results = []
    ncomp, lbl, stats, _ = cv2.connectedComponentsWithStats(merged)
    for ci in range(1, ncomp):
        x, y, w, h, _ = stats[ci]
        comp = (lbl[y:y + h, x:x + w] == ci)
        sub  = (denim[y:y + h, x:x + w] > 0) & comp
        n_mask = int(sub.sum())
        if n_mask == 0:
            continue
        area_m2 = n_mask * mpp * mpp
        pts = cv2.findNonZero(comp.astype(np.uint8))
        (_, _), (rw, rh), _ = cv2.minAreaRect(pts)
        L = max(rw, rh) * mpp - 2 * pad_m
        W = min(rw, rh) * mpp - 2 * pad_m
        if not (0.30 <= L <= 2.0 and 0.08 <= W <= 0.75
                and 0.02 <= area_m2 <= 0.8):
            continue
        # full-body centroid: include clothing pixels within ~0.35 m of the
        # denim (shirt/top of the same victim), else the estimate sits on
        # the legs instead of the torso
        ext = min(101, max(3, int(round(0.35 / mpp)) | 1))
        x0, y0 = max(0, x - ext), max(0, y - ext)
        x1 = min(frame.shape[1], x + w + ext)
        y1 = min(frame.shape[0], y + h + ext)
        roi = np.zeros((y1 - y0, x1 - x0), np.uint8)
        roi[y - y0:y - y0 + h, x - x0:x - x0 + w] = sub.astype(np.uint8)
        kext = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ext, ext))
        near = cv2.dilate(roi, kext) > 0
        body = roi.astype(bool) | ((cloth[y0:y1, x0:x1] > 0) & near)
        ys, xs = np.nonzero(body)
        wx, wy = pix_to_world(x0 + float(xs.mean()), y0 + float(ys.mean()),
                              H_pix_world)
        if abs(wx) < GRID_HALF_M and abs(wy) < GRID_HALF_M \
           and math.hypot(wx, wy) > ORIGIN_EXCL_M \
           and math.hypot(wx - drone_x, wy - drone_y) <= DET_MAX_RANGE_M:
            results.append((wx, wy))
    return results


def detect_victims_in_frame(frame, H_pix_world, drone_x=0.0, drone_y=0.0) -> tuple:
    """
    Return (denim_pts, broad_pts) — two lists of (wx, wy) world candidates.

    denim_pts (v6 detector above) is the primary signal; broad_pts keeps the
    old catch-any-colour blob detector as a fallback for worlds whose victims
    might not wear denim (clustering only uses it when denim comes up empty).

    Broad strategy (v4):
    - Three HSV layers: skin tones, any saturated colour, dark saturated.
    - Convexity filter rejects obviously fragmented contours.
    """
    mpp = _estimate_scale_mpp(H_pix_world)   # metres per pixel at current alt
    if mpp < 1e-6:
        return [], []

    denim_pts = _denim_candidates(frame, H_pix_world, mpp, drone_x, drone_y)

    # Expected human blob area in pixels²
    expected_area_px2 = math.pi * (HUMAN_DIAM_M / 2.0 / mpp) ** 2
    min_area = max(40.0, expected_area_px2 * HUMAN_AREA_MIN_F)
    max_area = expected_area_px2 * HUMAN_AREA_MAX_F

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Layer 1 – skin tones (face/arms visible from above, warm tones)
    mask  = cv2.inRange(hsv, np.array([ 0, 18,  50]), np.array([ 32, 255, 255]))
    # Layer 2 – any well-saturated colour (clothing of any hue)
    mask |= cv2.inRange(hsv, np.array([ 0, 45,  35]), np.array([180, 255, 235]))
    # Layer 3 – dark saturated blobs (dark hair, dark clothing with colour tint)
    mask |= cv2.inRange(hsv, np.array([ 0, 60,  10]), np.array([180, 255,  55]))

    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (min_area < area < max_area):
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1:
            continue
        circularity = 4 * math.pi * area / (perimeter ** 2)
        if circularity < CIRCULARITY_MIN:
            continue

        # Convexity: merged tile/furniture patterns often have re-entrant edges
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0 and area / hull_area < CONVEXITY_MIN:
            continue

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        wx, wy = pix_to_world(cx, cy, H_pix_world)

        # GRID_HALF_M (30 m), NOT HALF_M (15 m): the origin marker is a CORNER
        # takeoff mat, so a legal 30x30 m world can extend far past 15 m in one
        # direction. The 15 m box silently discarded every true-position
        # detection of medium's far victims once poses were scale-corrected
        # (raw detections 123 -> 27, victim2's cluster erased).
        if abs(wx) < GRID_HALF_M and abs(wy) < GRID_HALF_M:
            if math.hypot(wx, wy) > ORIGIN_EXCL_M:
                # Reject detections too far from the drone — these come from
                # projective distortion at image edges mapping to wrong world positions
                if math.hypot(wx - drone_x, wy - drone_y) <= DET_MAX_RANGE_M:
                    results.append((wx, wy))

    return denim_pts, results


# ══════════════════════════════════════════════════════════════════════════════
#  Clustering
# ══════════════════════════════════════════════════════════════════════════════

def cluster_detections(pts: list, r: float = None, div_r: float = None) -> list:
    """
    Greedy distance-based clustering with vote-count ranking and geographic
    diversity selection.

    Clusters are ranked by raw vote count; diversity selection then prevents a
    single high-density false-positive area (carpet, colourful furniture cluster)
    from filling all MAX_VICTIM_ESTIMATES slots — at most one position per
    DIVERSITY_R_M-metre neighbourhood is emitted, so victim positions in
    different areas of the world are represented even if they have fewer votes.
    """
    if not pts:
        return []
    if r is None:
        r = CLUSTER_R_M
    if div_r is None:
        div_r = DIVERSITY_R_M
    arr  = np.array([[p[0], p[1]] for p in pts], dtype=np.float64)
    wts  = np.array([p[2] if len(p) > 2 else 1.0 for p in pts])
    used = np.zeros(len(arr), bool)
    clusters = []
    for i in range(len(arr)):
        if used[i]:
            continue
        dists   = np.linalg.norm(arr - arr[i], axis=1)
        members = dists < r
        votes   = float(wts[members].sum())
        if votes < MIN_CLUSTER_VOTES:
            used[members] = True
            continue
        w = wts[members]
        centroid = tuple((arr[members] * w[:, None]).sum(axis=0) / w.sum())
        clusters.append((centroid, votes))
        used[members] = True
    clusters.sort(key=lambda c: c[1], reverse=True)
    top_votes = clusters[0][1] if clusters else 0.0

    # Geographic diversity: prevent one FP-dense area from taking all slots.
    # A quality floor gates slots beyond the old cap of 4 (see
    # MAX_VICTIM_ESTIMATES): a world with exactly 4 real victims already
    # exhausts its genuine votes on the top 4 clusters, so a weak 5th/6th
    # candidate is almost certainly a false positive, not a missed victim —
    # only admit it if it carries a real share of the top cluster's votes.
    QUALITY_FLOOR_FRAC = 0.35
    selected = []
    for centroid, votes in clusters:
        if len(selected) >= 4 and votes < QUALITY_FLOOR_FRAC * top_votes:
            continue
        too_close = any(
            math.hypot(centroid[0] - s[0], centroid[1] - s[1]) < div_r
            for s in selected
        )
        if not too_close:
            selected.append(centroid)
        if len(selected) >= MAX_VICTIM_ESTIMATES:
            break
    return selected


def cluster_denim_with_broad_fill(denim_pts, broad_pts):
    """Denim-primary clustering with broad-detector fill-in.

    Measured on the dumped detection sets (29/07):
      - denim clusters need >= 2 votes: single-frame denim blips are outliers
        (medium's (19.05,-5.85) sat 9.9 m from any victim);
      - slots beyond the 4th require a genuinely NEW area (>= 3 m from every
        selected estimate): recovers large's victim3 from its weak 2-vote
        clusters without emitting near-duplicates of already-covered victims;
      - broad clusters fill remaining slots up to 4 when denim covers fewer
        areas than that (medium: denim only sees 2 of 4 victims from cruise
        altitude; the 13-vote broad cluster near victim4 fills in), gated by
        a vote floor so sparse FP fields don't slip in.
    Small world is byte-identical under this rule (verified offline)."""
    def cwv(pts, r):
        arr = np.array([[p[0], p[1]] for p in pts], float)
        used = np.zeros(len(arr), bool)
        out = []
        for i in range(len(arr)):
            if used[i]:
                continue
            m = np.linalg.norm(arr - arr[i], axis=1) < r
            out.append((tuple(arr[m].mean(axis=0)), int(m.sum())))
            used |= m
        out.sort(key=lambda c: -c[1])
        return out

    sel = []
    for c, v in cwv(denim_pts, DENIM_CLUSTER_R):
        if v < 2:
            continue
        if len(sel) >= MAX_VICTIM_ESTIMATES:
            break
        sep = DENIM_DIVERSITY_R if len(sel) < 4 else EXTRA_SLOT_SEP_M
        if all(math.hypot(c[0] - s[0], c[1] - s[1]) >= sep for s in sel):
            sel.append(c)
    if broad_pts:
        bc = cwv(broad_pts, CLUSTER_R_M)
        top = bc[0][1]
        for c, v in bc:
            if len(sel) >= 4:
                break
            if v < max(3, 0.10 * top):
                continue
            if all(math.hypot(c[0] - s[0], c[1] - s[1]) >= DIVERSITY_R_M
                   for s in sel):
                sel.append(c)
    return sel


# ══════════════════════════════════════════════════════════════════════════════
#  Wall map
# ══════════════════════════════════════════════════════════════════════════════

class MapAccum:
    """
    Accumulates video frames into a 60 m world grid (0.05 m/px).

    Two composites are kept:
      min_gray — per-cell MINIMUM brightness across all views.  Walls are tall
                 (3 m) bright roughcast; their parallax smear moves between
                 viewpoints but their floor FOOTPRINT is occluded by the bright
                 wall from every viewpoint.  Taking the minimum erases the
                 view-dependent smear and keeps the view-independent footprint.
      mean/count — average colour mosaic (debug / future segmentation cues).
    """

    def __init__(self):
        self.min_gray = np.full((GRID_PX, GRID_PX), 255, np.uint8)
        self.sum_gray = np.zeros((GRID_PX, GRID_PX), np.float64)
        self.count    = np.zeros((GRID_PX, GRID_PX), np.uint16)
        self._mask    = None    # central-crop mask, built once per frame size

    def add(self, gray, H_pix_world):
        if abs(np.linalg.det(H_pix_world)) < 1e-12:
            return
        h, w = gray.shape[:2]
        if self._mask is None or self._mask.shape != (h, w):
            m = np.zeros((h, w), np.uint8)
            mh, mw = int(h * CROP_MARGIN_F), int(w * CROP_MARGIN_F)
            m[mh:h - mh, mw:w - mw] = 255
            self._mask = m
        # H_gs maps grid px -> source px; WARP_INVERSE_MAP makes warpPerspective
        # use it directly as the sampling map (verified empirically — passing it
        # without the flag smears each frame across the whole grid).
        H_gs = make_grid_to_src_H(H_pix_world)
        wg = cv2.warpPerspective(gray, H_gs, (GRID_PX, GRID_PX),
                                 flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        wm = cv2.warpPerspective(self._mask, H_gs, (GRID_PX, GRID_PX),
                                 flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        valid = wm >= 250
        np.minimum(self.min_gray, np.where(valid, wg, 255), out=self.min_gray)
        self.sum_gray[valid] += wg[valid]
        self.count[valid]    += 1


def _find_peaks_1d(prof, min_dist=12, rel_thr=SEG_REL_THR):
    """Local maxima above rel_thr * global max, greedy NMS by min_dist."""
    thr = rel_thr * prof.max()
    idx = [i for i in range(1, len(prof) - 1)
           if prof[i] >= thr and prof[i] >= prof[i - 1] and prof[i] >= prof[i + 1]]
    idx.sort(key=lambda i: -prof[i])
    picked = []
    for i in idx:
        if all(abs(i - j) >= min_dist for j in picked):
            picked.append(i)
    return sorted(picked)


def _lane_runs(img, p):
    """Support runs along the lane at coord p. Returns [(r0, r1, strength)]."""
    lane = img[:, max(p - SEG_BAND, 0):p + SEG_BAND + 1].mean(axis=1)
    pos = lane[lane > 0]
    if len(pos) == 0:
        return []
    on = lane > SEG_RUN_THR_F * np.percentile(pos, 90)
    idxs = np.nonzero(on)[0]
    if len(idxs) == 0:
        return []
    raw, run_start, prev = [], idxs[0], idxs[0]
    for y in idxs[1:]:
        if y - prev > SEG_GAP_PX:
            raw.append((run_start, prev))
            run_start = y
        prev = y
    raw.append((run_start, prev))
    out = []
    for r0, r1 in raw:
        if r1 - r0 < SEG_MIN_RUN:
            continue
        if on[r0:r1 + 1].mean() < SEG_MIN_DENS:
            continue
        out.append((r0, r1, float(lane[r0:r1 + 1].sum())))
    return out


def extract_wall_segments(spread):
    """
    Axis-aligned wall segments from the spread (mean-minus-min) mosaic.

    Returns {0: [[x, y0, y1, strength], …], 1: [[y, x0, x1, strength], …]}
    (0 = vertical walls, 1 = horizontal walls), already bbox-filtered,
    exterior-nudged and end-snapped.
    """
    segs = {0: [], 1: []}
    for axis in (0, 1):
        img = spread if axis == 0 else spread.T
        prof = cv2.GaussianBlur(img.sum(axis=0).reshape(1, -1), (0, 0), 2.0).ravel()
        if prof.max() <= 0:
            continue
        for p in _find_peaks_1d(prof):
            for r0, r1, s in _lane_runs(img, p):
                segs[axis].append([p, r0, r1, s])
    if not segs[0] or not segs[1]:
        return segs

    # Robust building bbox: extremes among lines carrying substantial evidence.
    # Threshold lowered from 0.25 -> 0.12: exterior walls sit farther from the
    # flight path so their parallax smear is systematically weaker even when
    # correctly detected, and excluding them here shrank the bbox used for
    # centring regardless of SEG_REL_THR (see docs/optimisation.md 07-12 log).
    def w_of(sl):
        return (sl[2] - sl[1]) * sl[3]
    strong_v = [s for s in segs[0] if w_of(s) >= 0.12 * max(w_of(t) for t in segs[0])]
    strong_h = [s for s in segs[1] if w_of(s) >= 0.12 * max(w_of(t) for t in segs[1])]
    bx0 = min(s[0] for s in strong_v); bx1 = max(s[0] for s in strong_v)
    by0 = min(s[0] for s in strong_h); by1 = max(s[0] for s in strong_h)

    keep = {0: [], 1: []}
    for axis, (lo, hi, plo, phi) in ((0, (bx0, bx1, by0, by1)),
                                     (1, (by0, by1, bx0, bx1))):
        for p, r0, r1, s in segs[axis]:
            if p < lo - SEG_BBOX_PAD or p > hi + SEG_BBOX_PAD:
                continue
            r0 = max(r0, plo - SEG_BBOX_PAD)
            r1 = min(r1, phi + SEG_BBOX_PAD)
            if r1 - r0 < SEG_MIN_RUN:
                continue
            keep[axis].append([p, r0, r1, s])
    segs = keep

    # Exterior lines: parallax smear biases the band centre outward (away from
    # the flight's orbit) — nudge the outermost lines back inward.
    for axis, lo, hi in ((0, bx0, bx1), (1, by0, by1)):
        for sl in segs[axis]:
            if abs(sl[0] - lo) <= 3:
                sl[0] += SEG_NUDGE_PX
            elif abs(sl[0] - hi) <= 3:
                sl[0] -= SEG_NUDGE_PX

    # Walls terminate at other walls: snap run ends to perpendicular lines.
    for axis in (0, 1):
        perp = segs[1 - axis]
        for sl in segs[axis]:
            for end in (1, 2):
                cands = [q[0] for q in perp
                         if q[1] - SEG_SNAP_PX <= sl[0] <= q[2] + SEG_SNAP_PX
                         and abs(q[0] - sl[end]) <= SEG_SNAP_PX]
                if cands:
                    sl[end] = min(cands, key=lambda c: abs(c - sl[end]))
    return segs


def finalize_wall_map(accum: "MapAccum") -> np.ndarray:
    """
    Extract walls from the mosaic and emit the final 600x600 map, centred on
    the walls' bounding box (the marking supervisor centres its ground truth
    the same way).  Returns uint8: 0 = wall, 255 = free.

    Walls are found via their parallax smear: 3 m walls are the strongest
    view-dependent structures, so the mean-minus-min composite highlights them
    as bands, and long axis-aligned walls integrate into strong row/column
    projection peaks (see extract_wall_segments).
    """
    visited = accum.count > 0
    if not visited.any():
        print("[map] WARNING: nothing accumulated — returning all-free map")
        return np.full((MAP_PX, MAP_PX), 255, np.uint8)

    meang = np.where(visited,
                     accum.sum_gray / np.maximum(accum.count, 1), 0).astype(np.float32)
    spread = np.clip(meang - accum.min_gray.astype(np.float32), 0, 255)
    spread = np.clip(spread, 0, np.percentile(spread[visited], 98))
    spread[~visited] = 0.0
    spread = cv2.GaussianBlur(spread, (0, 0), 2.0)

    if os.environ.get("SAR_DUMP_SPREAD"):
        _stem = os.path.splitext(os.path.basename(_current_recording or "spread"))[0]
        np.save(os.path.join(SIM_LOGS, f"spread_{_stem}.npy"), spread)

    segs = extract_wall_segments(spread)

    # When the poses were scaled up (high-altitude worlds), the same wall
    # evidence is spread over a larger mosaic, spawning extra weak spurious
    # segments — prune them.  Worlds with no scale correction are left as-is
    # (pruning there only removes genuine faint walls; see docs/optimisation).
    if _LAST_MAP_SCALE > 1.5:
        n_before = len(segs[0]) + len(segs[1])
        segs = _prune_weak_segments(segs, 0.15)
        print(f"[map] pruned weak segments (map scaled ×{_LAST_MAP_SCALE:.2f}): "
              f"{n_before} → {len(segs[0]) + len(segs[1])}")

    n_segs = len(segs[0]) + len(segs[1])
    if len(segs[0]) < 2 or len(segs[1]) < 2:
        print(f"[map] WARNING: only {n_segs} wall segments found — returning all-free map")
        return np.full((MAP_PX, MAP_PX), 255, np.uint8)

    grid = np.zeros((GRID_PX, GRID_PX), np.uint8)
    t = WALL_THICK_PX // 2
    for x, y0, y1, _ in segs[0]:
        grid[min(y0, y1):max(y0, y1) + 1, max(x - t, 0):x + t] = 255
    for y, x0, x1, _ in segs[1]:
        grid[max(y - t, 0):y + t, min(x0, x1):max(x0, x1) + 1] = 255

    # Save the origin-anchored wall grid for robot navigation.  Unlike the
    # bbox-centred deliverable PNG, this grid keeps a fixed, known mapping:
    # mat-frame origin (0,0) sits at pixel (GRID_PX/2, GRID_PX/2), +X → +col,
    # +Y → -row, at MAP_SCALE m/px.  The controller loads it to plan routes.
    try:
        np.save(os.path.join(SIM_LOGS, "nav_grid.npy"), grid)
    except Exception as e:
        print(f"[map] WARNING: could not save nav_grid.npy: {e}")

    # Centre the 600x600 output window on the wall bounding box (GT convention).
    ys, xs = np.nonzero(grid)
    cx = int(round((xs.min() + xs.max()) / 2.0))
    cy = int(round((ys.min() + ys.max()) / 2.0))
    x0 = min(max(cx - MAP_PX // 2, 0), GRID_PX - MAP_PX)
    y0 = min(max(cy - MAP_PX // 2, 0), GRID_PX - MAP_PX)
    crop = grid[y0:y0 + MAP_PX, x0:x0 + MAP_PX]

    wall_map = np.full((MAP_PX, MAP_PX), 255, np.uint8)
    wall_map[crop > 0] = 0

    pct_wall = 100.0 * (wall_map == 0).sum() / wall_map.size
    print(f"[map] Map built: {pct_wall:.2f}% wall px  ({len(segs[0])}V+{len(segs[1])}H "
          f"segments, window origin grid px ({x0},{y0}))")
    return wall_map


# ══════════════════════════════════════════════════════════════════════════════
#  IMU metric scale alignment (Option A)
# ══════════════════════════════════════════════════════════════════════════════
#
# Visual odometry pins metric scale only via the ArUco markers, which exist
# only on the origin mat.  Off-mat, position is propagated by relative image
# steps at the mat's metres-per-pixel — so when the drone climbs to cruise the
# true m/px grows but the pipeline keeps the mat-altitude scale, compressing
# every off-mat point (walls + victims) toward the origin.  The IMU horizontal
# trajectory is metric and altitude-independent (this recording accelerates
# along heading only, acc_y≈0, heading known from the compass), so it supplies
# the global scale/orientation the vision solve is missing.  We fit a single
# 2-D similarity (scale + rotation + translation) mapping the visual drone
# track onto the IMU track and premultiply every pose by it — correcting
# victims and the map in one step.

# Scale factor applied by the last _imu_scale_align call (1.0 = no correction).
# Consumed by finalize_wall_map to decide whether the mosaic was inflated and
# therefore needs weak-segment pruning.
_LAST_MAP_SCALE = 1.0


def _fit_radial_scale(V, I):
    """Least-squares isotropic scale s minimising ||s·V − I|| about the origin."""
    num = float((V * I).sum())
    den = float((V * V).sum())
    return num / den if den > 1e-9 else None


def _prune_weak_segments(segs, frac):
    """Drop wall segments whose length×strength is below `frac` of the strongest
    in their axis (never empties a whole axis).  Cuts spurious weak segments the
    scale expansion spreads across the inflated mosaic."""
    if frac <= 0:
        return segs
    out = {0: [], 1: []}
    for axis in (0, 1):
        if not segs[axis]:
            continue
        wv = [(s[2] - s[1]) * s[3] for s in segs[axis]]
        mx = max(wv)
        out[axis] = [s for s, w in zip(segs[axis], wv) if w >= frac * mx]
        if not out[axis]:
            out[axis] = [segs[axis][int(np.argmax(wv))]]
    return out


def _imu_scale_align(poses, imu_path, fps, total, w_px, h_px):
    """Correct visual-odometry scale compression using the metric IMU track.

    The diagnosed failure is a pure radial compression toward the ArUco origin
    (mat-altitude m/px applied at cruise altitude), so orientation and origin
    are already correct — only the global scale drifts, and only ever *too
    small* (cruise altitude ≥ mat altitude ⇒ true m/px ≥ mat m/px).  We fit a
    single robust isotropic scale about the origin against the IMU horizontal
    trajectory and apply it only when it expands (s ≥ 1); anything else is the
    fit chasing IMU drift on a world whose vision was already correct, so we
    leave those poses untouched.  Full no-op when IMU data is missing.
    """
    global _LAST_MAP_SCALE
    _LAST_MAP_SCALE = 1.0
    if not imu_path or not os.path.exists(imu_path) or len(poses) < 20:
        return poses
    traj = _integrate_imu_trajectory(imu_path, fps, total, poses[0][0])
    if not traj:
        print("[scale] IMU trajectory unavailable — keeping visual poses")
        return poses

    tkeys = np.array(sorted(traj.keys()))
    V, I, F = [], [], []
    for fi, H in poses:
        vx, vy = pix_to_world(w_px / 2.0, h_px / 2.0, H)
        k = int(tkeys[np.argmin(np.abs(tkeys - fi))])
        if abs(k - fi) > fps:                          # no IMU sample within 1 s
            continue
        ix, iy, _, _ = traj[k]
        V.append((vx, vy)); I.append((ix, iy)); F.append(fi)
    if len(V) < 20:
        return poses

    V = np.asarray(V, float); I = np.asarray(I, float)
    if os.environ.get("SAR_DEBUG"):
        _dbg = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "sim_logs", "debug")
        os.makedirs(_dbg, exist_ok=True)
        np.savez(os.path.join(_dbg, "scale_pairs.npz"),
                 V=V, I=I, fi=np.asarray(F, float))
        print(f"[scale] debug: dumped {len(V)} visual/IMU pose pairs")
    # Global fit first — used ONLY as a guard.  s ≤ 1.02 means vision is not
    # compressed (small-world case): keep the historical no-op path unchanged.
    Vt, It = V, I
    s = _fit_radial_scale(Vt, It)                      # robust: 2 trim passes
    for _ in range(2):
        if s is None:
            return poses
        res = np.linalg.norm(s * Vt - It, axis=1)
        med = float(np.median(res))
        keep = res <= max(2.5 * med, 0.5)
        if keep.all() or int(keep.sum()) < max(15, int(0.4 * len(Vt))):
            break
        Vt, It = Vt[keep], It[keep]
        s = _fit_radial_scale(Vt, It)
    if s is None:
        return poses

    if s <= 1.02:
        print(f"[scale] IMU radial scale {s:.3f} ≤ 1 — vision not compressed, "
              f"no correction")
        return poses

    # Per-sample radial correction.  A single global scale cannot fix the
    # measured warp (implied gain falls from ~7x near origin to ~2x at the far
    # edge on medium world), so give every posed sample its own radial gain
    # g_i = |IMU_i| / |visual_i| about the origin: ArUco-anchored on-mat
    # samples (both radii agree) get g≈1 and stay put, compressed cruise
    # samples are expanded by their true factor.  Median-smoothed over time to
    # reject IMU dead-reckoning noise; interpolated by frame index for poses
    # without an IMU pairing.
    # IMPORTANT: the gain corrects each sample's POSITION only.  The local
    # metres-per-pixel (linear part of H) stays at the single global scale —
    # the victim detector's altitude classification and size gates key off
    # mpp, and per-sample mpp scrambling was measured to gut detection
    # (123 → 33 raw detections on medium).
    rV = np.hypot(V[:, 0], V[:, 1])
    rI = np.hypot(I[:, 0], I[:, 1])
    g = np.ones(len(V))
    m = rV > 0.75                                      # below: on-mat, keep g=1
    g[m] = rI[m] / rV[m]
    g = np.clip(g, 1.0, 8.0)
    gs = np.array([np.median(g[max(0, i - 4):i + 5]) for i in range(len(g))])
    Fs = np.asarray(F, float)
    order = np.argsort(Fs)
    Fs, gs = Fs[order], gs[order]
    s_med = float(np.median(gs))
    _LAST_MAP_SCALE = s_med
    s_lin = min(s, 3.5)                                # linear part: global fit
    Sg = np.diag([s_lin, s_lin, 1.0]).astype(np.float64)
    print(f"[scale] IMU per-sample radial correction: median ×{s_med:.3f}  "
          f"range {gs.min():.2f}–{gs.max():.2f}  ({len(V)} pairs, "
          f"global fit ×{s:.3f}, linear ×{s_lin:.3f})")
    out = []
    for fi, H in poses:
        gi = float(np.interp(float(fi), Fs, gs))
        cx, cy = pix_to_world(w_px / 2.0, h_px / 2.0, H)
        T = np.array([[1.0, 0.0, (gi - s_lin) * cx],
                      [0.0, 1.0, (gi - s_lin) * cy],
                      [0.0, 0.0, 1.0]])
        out.append((fi, T @ Sg @ H))
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  Main analysis pipeline
# ══════════════════════════════════════════════════════════════════════════════

def analyse(video_path: str, imu_path: str = None) -> tuple:
    """
    Full offline analysis of a flyover video.

    Returns:
        victims  – [(x, y), ...] in world metres
        wall_map – 600x600 uint8 array (0=wall, 255=free)
    """
    global _current_recording
    _current_recording = video_path

    cap   = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vid_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  or 1920.0
    vid_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 1080.0
    cap.release()

    print(f"[video] {os.path.basename(video_path)}  "
          f"{total} frames @ {fps:.1f} fps  ({total/fps:.1f} s)")

    # ── Phase 0: camera intrinsic calibration ─────────────────────────────────
    print("[intrinsics] Calibrating camera from flyover video…")
    K_cam, dist_cam = calibrate_intrinsics(video_path, fps, total)
    _do_undistort = (K_cam is not None and dist_cam is not None and
                     np.any(np.abs(dist_cam) > 1e-6))
    if K_cam is not None and not _do_undistort:
        print("[intrinsics] Distortion is negligible — K stored, skipping per-frame undistort")

    # ── Phase 1: vision pose tracking (PASS 1) ────────────────────────────────
    yaw = _load_compass_yaw(imu_path)
    print(f"[track] PASS 1 — visual odometry at {TRACK_HZ} Hz"
          f" ({'compass-constrained' if yaw is not None else 'vision-only'})…")
    poses, n_samples = track_poses(video_path, fps, total, K_cam, dist_cam, yaw)
    coverage = len(poses) / max(n_samples, 1)

    if coverage < MIN_TRACK_COVER:
        print(f"[track] WARNING — only {coverage:.0%} of samples posed; "
              f"falling back to IMU dead reckoning")
        poses = _imu_poses(video_path, imu_path, fps, total, K_cam)

    if not poses:
        print("[track] WARNING — no poses at all; emitting empty outputs")
        return [], np.full((MAP_PX, MAP_PX), 255, np.uint8)

    # ── Phase 1.5: pose refinement against the consensus mosaic ───────────────
    print("[refine] PASS 1.5 — re-registering samples against the mosaic…")
    poses = refine_poses_mosaic(video_path, poses, _do_undistort, K_cam, dist_cam)

    # ── Phase 1.6: global metric scale alignment to the IMU track (Option A) ──
    poses = _imu_scale_align(poses, imu_path, fps, total, vid_w, vid_h)

    if os.environ.get("SAR_DUMP_POSES") and poses:
        _stem = os.path.splitext(os.path.basename(_current_recording or "poses"))[0]
        _fis = np.array([fi for fi, _ in poses])
        _dxy = np.array([pix_to_world(vid_w / 2.0, vid_h / 2.0, H) for _, H in poses])
        _traj = _integrate_imu_trajectory(imu_path, fps, total, poses[0][0]) or {}
        _ik = np.array(sorted(_traj)) if _traj else np.zeros(0)
        _ixy = np.array([_traj[k][:2] for k in sorted(_traj)]) if _traj else np.zeros((0, 2))
        np.savez(os.path.join(SIM_LOGS, f"poses_{_stem}.npz"),
                 fi=_fis, dxy=_dxy, imu_fi=_ik, imu_xy=_ixy)

    pose_by_fi = {fi: H for fi, H in poses}

    # Footprint spans: samples whose span sits far from the cruise median are
    # low-altitude victim dives (or takeoff) — their views are oblique, so
    # they would smear walls across the mosaic.  Victims are still detected
    # in them (that is where victims are filmed close-up); they just don't
    # contribute to the map.
    def _span(H, w_px=1920.0, h_px=1080.0):
        a = H @ np.array([0.0, h_px / 2.0, 1.0])
        b = H @ np.array([w_px, h_px / 2.0, 1.0])
        return float(np.linalg.norm(b[:2] / b[2] - a[:2] / a[2]))

    spans_by_fi = {fi: _span(H) for fi, H in poses}
    med_span = float(np.median(list(spans_by_fi.values())))

    # ── Phase 2: victim detection + map accumulation (PASS 2) ─────────────────
    print("[track] PASS 2 — victim detection + map accumulation…")
    victim_pts       = []
    broad_victim_pts = []
    accum      = MapAccum()
    n_dive     = 0

    cap = cv2.VideoCapture(video_path)
    fi  = -1
    while True:
        if not cap.grab():
            break
        fi += 1
        H_pix_world = pose_by_fi.get(fi)
        if H_pix_world is None:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            break
        if _do_undistort:
            frame = cv2.undistort(frame, K_cam, dist_cam)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h_px, w_px = gray.shape[:2]
        is_dive = not (0.75 * med_span
                       <= _span(H_pix_world, w_px, h_px) <= 1.35 * med_span)
        if is_dive:
            n_dive += 1
        else:
            accum.add(gray, H_pix_world)

        # Drone nadir in world coords = principal point through H (for the
        # long-range rejection gate in detect_victims_in_frame).
        h_img, w_img = gray.shape[:2]
        nx, ny = pix_to_world(w_img / 2.0, h_img / 2.0, H_pix_world)

        # Per-frame de-duplication: collapse detections within CLUSTER_R_M of
        # each other so one frame contributes at most one vote per location.
        denim_pts, broad_pts = detect_victims_in_frame(frame, H_pix_world, nx, ny)
        for frame_pts, store, dedup_r in ((denim_pts, victim_pts, DENIM_CLUSTER_R),
                                          (broad_pts, broad_victim_pts, CLUSTER_R_M)):
            if not frame_pts:
                continue
            # Per-frame de-dup: one vote per dedup_r neighbourhood.
            fp_arr  = np.array(frame_pts, dtype=np.float64)
            fp_used = np.zeros(len(fp_arr), bool)
            for i in range(len(fp_arr)):
                if fp_used[i]:
                    continue
                nearby = np.linalg.norm(fp_arr - fp_arr[i], axis=1) < dedup_r
                cx_w, cy_w = fp_arr[nearby].mean(axis=0)
                store.append((float(cx_w), float(cy_w)))
                fp_used[nearby] = True
    cap.release()
    print(f"[detect] {len(victim_pts)} denim / {len(broad_victim_pts)} broad raw "
          f"detections across {len(pose_by_fi)} posed samples"
          f"  ({n_dive} low-altitude samples excluded from map)")

    if os.environ.get("SAR_DUMP_DETS"):
        _stem = os.path.splitext(os.path.basename(_current_recording or "dets"))[0]
        np.save(os.path.join(SIM_LOGS, f"dets_denim_{_stem}.npy"),
                np.array(victim_pts, dtype=np.float64).reshape(-1, 2))
        np.save(os.path.join(SIM_LOGS, f"dets_broad_{_stem}.npy"),
                np.array(broad_victim_pts, dtype=np.float64).reshape(-1, 2))

    # ── Phase 3: cluster victim detections ────────────────────────────────────
    # Denim detections are high-precision; only fall back to the broad blob
    # detector when denim finds (almost) nothing.
    if len(victim_pts) >= 6:
        victims = cluster_denim_with_broad_fill(victim_pts, broad_victim_pts)
    else:
        print("[detect] denim detector sparse — falling back to broad detector")
        victims = cluster_detections(broad_victim_pts)
    print(f"[detect] {len(victims)} confirmed victim estimates")

    # ── Phase 4: build wall map ───────────────────────────────────────────────
    print("[map] Building wall map…")
    if SAR_DEBUG:
        _dump_debug(accum, poses, victim_pts)
    wall_map = finalize_wall_map(accum)

    return victims, wall_map


def _imu_poses(video_path, imu_path, fps, total, K_cam):
    """Legacy IMU dead-reckoning poses (fallback when vision tracking fails)."""
    calib_result = _scan_for_best_calibration(video_path, fps, total)
    if calib_result is None:
        cap = cv2.VideoCapture(video_path)
        _, frame0 = cap.read()
        cap.release()
        if frame0 is None:
            return []
        H_calib, calib_fi = fallback_H(frame0), 0
    else:
        H_calib, calib_fi, _ = calib_result

    if K_cam is not None:
        cx0, cy0 = float(K_cam[0, 2]), float(K_cam[1, 2])
    else:
        cap = cv2.VideoCapture(video_path)
        cx0 = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  / 2.0
        cy0 = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2.0
        cap.release()

    drone_traj = _integrate_imu_trajectory(imu_path, fps, total, calib_fi)
    if not drone_traj:
        return [(calib_fi, H_calib)]

    nearest_fi = min(drone_traj.keys(), key=lambda k: abs(k - calib_fi))
    _, _, h0, theta_0 = drone_traj[nearest_fi]
    h0 = max(0.1, h0)
    traj_keys = sorted(drone_traj.keys())

    poses = []
    sample_every = max(1, int(round(fps / TRACK_HZ)))
    import bisect
    for fi in range(0, total, sample_every):
        idx = bisect.bisect_left(traj_keys, fi)
        if idx >= len(traj_keys):
            idx = len(traj_keys) - 1
        elif idx > 0 and abs(traj_keys[idx - 1] - fi) < abs(traj_keys[idx] - fi):
            idx -= 1
        dx, dy, dh, dtheta = drone_traj[traj_keys[idx]]
        H = _pose_to_H(dx, dy, dh, dtheta, theta_0, h0, H_calib, cx0, cy0)
        if abs(np.linalg.det(H)) > 1e-12:
            poses.append((fi, H))
    return poses


def _dump_debug(accum: "MapAccum", poses=None, victim_pts=None):
    """Write mosaic diagnostics + raw arrays (for fast --segment-only tuning)."""
    dbg = os.path.join(SIM_LOGS, "debug")
    os.makedirs(dbg, exist_ok=True)
    mean = (accum.sum_gray / np.maximum(accum.count, 1)).astype(np.uint8)
    cv2.imwrite(os.path.join(dbg, "mosaic_min.png"),  accum.min_gray)
    cv2.imwrite(os.path.join(dbg, "mosaic_mean.png"), mean)
    cv2.imwrite(os.path.join(dbg, "coverage.png"),
                np.clip(accum.count * 8, 0, 255).astype(np.uint8))
    extra = {}
    if poses:
        extra["pose_fi"] = np.array([fi for fi, _ in poses])
        extra["pose_H"]  = np.stack([H for _, H in poses])
    if victim_pts:
        extra["victim_pts"] = np.array(victim_pts)
    np.savez_compressed(os.path.join(dbg, "accum.npz"),
                        min_gray=accum.min_gray,
                        sum_gray=accum.sum_gray,
                        count=accum.count, **extra)
    print(f"[debug] mosaics + accum.npz written to {dbg}")


# ══════════════════════════════════════════════════════════════════════════════
#  Save outputs
# ══════════════════════════════════════════════════════════════════════════════

def save(victims: list, wall_map: np.ndarray):
    """Write victim CSV, wall map PNG, and cache-key metadata to sim_logs/."""
    os.makedirs(SIM_LOGS, exist_ok=True)

    with open(CSV_OUT, "w", newline="") as f:
        for x, y in victims:
            f.write(f"{x:.4f},{y:.4f}\n")
    print(f"[save] {CSV_OUT}  ({len(victims)} entries)")

    _, bw  = cv2.threshold(wall_map, 127, 255, cv2.THRESH_BINARY)
    rgb    = cv2.cvtColor(bw, cv2.COLOR_GRAY2RGB)
    Image.fromarray(rgb).save(MAP_OUT)
    print(f"[save] {MAP_OUT}")

    # Write cache key (recording path + code stamp) so the next run skips
    # reanalysis only when BOTH the recording and this code are unchanged.
    if _current_recording:
        with open(_LAST_REC, "w") as f:
            f.write(f"{_current_recording}\n{_code_stamp()}\n")


# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="SAR flyover video analyser — victim CSV + wall map PNG")
    ap.add_argument("--video", required=False, default=None,
                    help="Path to flyover .mp4")
    ap.add_argument("--imu", default=None,
                    help="Path to matching IMU .csv (reserved for future use)")
    ap.add_argument("--segment-only", action="store_true",
                    help="Dev: rebuild map_estimate.png from sim_logs/debug/accum.npz "
                         "(skips video decoding; requires a prior SAR_DEBUG=1 run)")
    args = ap.parse_args()

    if args.segment_only:
        npz = np.load(os.path.join(SIM_LOGS, "debug", "accum.npz"))
        accum = MapAccum()
        accum.min_gray = npz["min_gray"]
        accum.sum_gray = npz["sum_gray"]
        accum.count    = npz["count"]
        wall_map = finalize_wall_map(accum)
        _, bw = cv2.threshold(wall_map, 127, 255, cv2.THRESH_BINARY)
        Image.fromarray(cv2.cvtColor(bw, cv2.COLOR_GRAY2RGB)).save(MAP_OUT)
        print(f"[save] {MAP_OUT} (segment-only)")
        return

    if not args.video:
        ap.error("--video is required (unless --segment-only)")

    # Auto-detect IMU CSV alongside the video if --imu not given
    imu_path = args.imu
    if imu_path is None:
        candidate = os.path.splitext(os.path.abspath(args.video))[0] + ".csv"
        if os.path.exists(candidate):
            imu_path = candidate
            print(f"[main] Auto-detected IMU: {os.path.basename(imu_path)}")

    victims, wall_map = analyse(os.path.abspath(args.video), imu_path)
    save(victims, wall_map)
    print("[done]")


if __name__ == "__main__":
    main()
