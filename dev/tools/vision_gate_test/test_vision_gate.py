"""
Offline verification of the hardened victim-detection gate.

Feeds synthetic RGB + depth frames through the REAL _camera_victim_confidence()
in controllers/proposed_solution/proposed_solution.py (imported against the stub
`controller` module in this folder) and asserts the outcome for each scenario
that mattered in the 28/07 mission logs.

Run it before every Webots run - it takes under a second and catches a broken
detection gate without spending a 3-minute simulation:

    .venv/bin/python tools/vision_gate_test/test_vision_gate.py

No Webots, no flyover recordings and no venv-specific paths required beyond
numpy + opencv, which the controller already needs.

Camera model: 640x480, hfov 1.047 rad, optical centre 0.19 m above the floor,
level. Vertical half-FOV works out to ~23.4 deg, so f_y = 240/tan(23.4) ~= 555.
"""
import importlib.util
import math
import os
import sys

import numpy as np
# Pre-import cv2 from the CURRENT interpreter's environment. The controller
# module prepends the repo .venv to sys.path when it loads; if that venv was
# built for another platform (e.g. macOS venv, Linux test runner) its cv2
# binary won't load. With cv2 already in sys.modules the controller's own
# `import cv2` is a cache hit and the venv shim is harmlessly ignored.
import cv2  # noqa: F401


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
TARGET = os.path.join(REPO, "controllers", "proposed_solution", "proposed_solution.py")

# Stubs first: `controller` and `prepare_mission` must resolve to this folder,
# never to Webots' real controller module or the real preflight pipeline.
sys.path.insert(0, HERE)


def _load_controller_module():
    """Import proposed_solution.py by path, WITHOUT putting its directory on
    sys.path — otherwise the real prepare_mission would shadow the stub and the
    test would kick off a full flyover analysis."""
    if not os.path.isfile(TARGET):
        raise SystemExit("cannot find %s" % TARGET)
    spec = importlib.util.spec_from_file_location("proposed_solution_under_test",
                                                  TARGET)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ps = _load_controller_module()
from controller import Robot            # noqa: E402  (the stub in this folder)

W, H = 640, 480
HFOV = 1.047
VFOV = 2 * math.atan(math.tan(HFOV / 2) * H / W)
F_Y = (H / 2) / math.tan(VFOV / 2)
CAM_H = ps.CAM_HEIGHT_M

DENIM_BGR = (150, 60, 20)     # HSV ~ (110, 220, 150) -> inside the denim window
WARM_BGR = (90, 150, 210)     # HSV ~ (15, 145, 210)  -> inside the warm/clothing mask


def row_for_height(height_m, depth_m):
    """Image row at which a point `height_m` above the floor appears, at range."""
    elev = math.atan((height_m - CAM_H) / depth_m)
    return int(round(H / 2 - F_Y * math.tan(elev)))


def make_frame(denim_boxes, warm_frac=0.10):
    """BGRA buffer: denim rectangles plus enough warm pixels to pass `ratio`."""
    img = np.zeros((H, W, 3), np.uint8)
    img[:, :] = (40, 40, 40)                       # neutral dark background
    warm_rows = int(H * warm_frac)
    img[H - warm_rows:, :] = WARM_BGR              # warm band along the bottom
    for (r0, r1, c0, c1) in denim_boxes:
        img[max(0, r0):max(0, r1), c0:c1] = DENIM_BGR
    bgra = np.dstack([img, np.full((H, W, 1), 255, np.uint8)])
    return bgra.tobytes()


def make_depth(default, patches=()):
    """Flat depth list; `patches` = [(r0, r1, c0, c1, value), ...]."""
    d = np.full((H, W), default, np.float32)
    for (r0, r1, c0, c1, v) in patches:
        d[max(0, r0):max(0, r1), c0:c1] = v
    return d.reshape(-1).tolist()


def make_depth_ramp(rows_span, depth_m, cols):
    """Depth for a vertical object at constant range (a plane facing us)."""
    d = np.full((H, W), float("inf"), np.float32)
    r0, r1 = rows_span
    d[max(0, r0):max(0, r1), cols[0]:cols[1]] = depth_m
    return d.reshape(-1).tolist()


def gate(robot, bot, rgb, depth):
    robot.camera.buf = rgb
    robot.depth.buf = depth
    conf, dep, off = bot._camera_victim_confidence()
    return conf, dep, off, dict(bot._last_vision)


def band_rows(h_lo, h_hi, depth_m):
    """Rows spanned by an object between two heights, clipped to the frame."""
    r_top = row_for_height(h_hi, depth_m)
    r_bot = row_for_height(h_lo, depth_m)
    return max(0, min(r_top, r_bot)), min(H, max(r_top, r_bot))


results = []


def check(name, ok, detail):
    results.append((ok, name, detail))
    print(("  PASS  " if ok else "  FAIL  ") + name)
    print("          " + detail)


def main():
    robot = Robot("robot1")
    bot = ps.SARRobot(robot)
    bot.x, bot.y = 6.0, 6.0          # well clear of the origin FP zone
    print("\nCamera model: vfov=%.1f deg  f_y=%.1f  cam_h=%.2f m\n"
          % (math.degrees(VFOV), F_Y, CAM_H))

    # ── 1. Victim at report range: jeans 0.2-0.9 m, 0.70 m away ─────────────
    r0, r1 = band_rows(0.20, 0.90, 0.70)
    rgb = make_frame([(r0, r1, 240, 400)])
    dep = make_depth(float("inf"), [(r0, r1, 240, 400, 0.70)])
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("victim at 0.70 m passes the gate",
          v["why"] == "candidate" and conf >= 0.15 and abs(d - 0.70) < 0.05,
          "why=%s conf=%.2f depth=%.2f h_med=%.2f h_low=%.2f"
          % (v["why"], conf, d, v.get("med_h", float("nan")),
             v.get("low_h", float("nan"))))

    # ── 2. Victim detectable at homing range (3.0 m) ────────────────────────
    r0, r1 = band_rows(0.20, 1.10, 3.0)
    rgb = make_frame([(r0, r1, 280, 360)])
    dep = make_depth(float("inf"), [(r0, r1, 280, 360, 3.0)])
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("victim at 3.0 m enters homing",
          v["why"] == "candidate" and conf >= ps.APPROACH_ENTER_CONF
          and d <= ps.APPROACH_ENTER_M,
          "why=%s conf=%.2f depth=%.2f (enter needs conf>=%.2f, depth<=%.1f)"
          % (v["why"], conf, d, ps.APPROACH_ENTER_CONF, ps.APPROACH_ENTER_M))

    # ── 3. Window glass, 1.0-1.8 m high, 3.0 m away ─────────────────────────
    r0, r1 = band_rows(1.00, 1.80, 3.0)
    rgb = make_frame([(r0, r1, 200, 440)])
    dep = make_depth(float("inf"), [(r0, r1, 200, 440, 3.0)])
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("window glass at 3.0 m is rejected",
          conf == 0.0 and v["why"] == "denim-too-high",
          "why=%s h_med=%.2f h_low=%.2f (limits med<=%.2f low<=%.2f)"
          % (v["why"], v.get("med_h", float("nan")), v.get("low_h", float("nan")),
             ps.VICTIM_DENIM_MAX_H_M, ps.VICTIM_DENIM_LOW_H_M))

    # ── 4. Open sky: blue with no finite depth ──────────────────────────────
    rgb = make_frame([(0, 120, 180, 460)])
    dep = make_depth(float("inf"))
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("sky with no finite depth is rejected",
          conf == 0.0 and v["why"] == "denim-no-depth",
          "why=%s" % v["why"])

    # ── 5. Scattered speckle: many tiny blue dots ───────────────────────────
    boxes = [(40 + 17 * i, 43 + 17 * i, 30 + 21 * i, 33 + 21 * i) for i in range(25)]
    rgb = make_frame(boxes)
    dep = make_depth(0.5)
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("scattered blue speckle is rejected",
          conf == 0.0 and v["why"] == "denim-blob-small",
          "why=%s largest blob frac=%.5f (needs >=%.4f)"
          % (v["why"], v.get("blob", 0.0), ps.DENIM_MIN_COMPONENT_FRAC))

    # ── 6. THE REGRESSION: far denim + one stray near blue pixel ────────────
    # Old code took np.min over the whole denim mask, so the stray pixel put
    # min_depth at 0.40 m and tripped the <0.80 m report gate from 3 m away.
    r0, r1 = band_rows(0.20, 1.10, 3.0)
    rgb = make_frame([(r0, r1, 280, 360), (300, 303, 10, 13)])
    dep = make_depth(float("inf"),
                     [(r0, r1, 280, 360, 3.0), (300, 303, 10, 13, 0.40)])
    conf, d, off, v = gate(robot, bot, rgb, dep)
    old_min = 0.40
    check("stray near-pixel no longer collapses the range",
          d > ps.APPROACH_STOP_M and abs(d - 3.0) < 0.2,
          "median depth=%.2f (np.min would have given %.2f, tripping the %.2f m "
          "report gate)" % (d, old_min, ps.APPROACH_STOP_M))

    # ── 6b. THE 17:32 DECOY: small blue object at floor level, close range ──
    # An ~8x8 cm blue object at 0.81 m passed blob-fraction and height gates
    # (both scale with distance) and fired the run's only false report. The
    # physical-area gate (px x depth^2 / focal^2) is scale-invariant: 0.007 m2
    # can never be trousers, however close the robot parks.
    side_px = int(round(0.085 * F_Y / 0.81))         # 8.5 cm at 0.81 m
    r0, r1 = band_rows(0.05, 0.05 + 0.085, 0.81)
    rgb = make_frame([(r0, r0 + side_px, 300, 300 + side_px)])
    dep = make_depth(float("inf"), [(r0, r0 + side_px, 300, 300 + side_px, 0.81)])
    conf, d, off, v = gate(robot, bot, rgb, dep)
    check("small blue decoy at 0.81 m is rejected",
          conf == 0.0 and v["why"] == "denim-area-small",
          "why=%s blob=%.4f depth=%.2f (area gate needs >=%.3f m2)"
          % (v["why"], v.get("blob", 0.0), v.get("depth", float("nan")),
             ps.VICTIM_DENIM_MIN_AREA_M2))

    # ── 7. Report requires a streak, and does not retire a distant waypoint ─
    bot._my_waypoints = [(20.0, 20.0)]        # target far from the sighting
    bot.target_idx = 0
    bot._homing = True
    bot._home_start_time = robot.getTime()
    bot._report_streak = 0
    bot._reported_positions = []
    # 0.50 m (inside the APPROACH_STOP_M=0.60 park gate); at close range a
    # lying victim spans most of the frame width, so the synthetic denim is
    # wide enough (320 px) that the clipped visible area stays person-sized.
    r0, r1 = band_rows(0.20, 0.90, 0.50)
    robot.camera.buf = make_frame([(r0, r1, 160, 480)])
    robot.depth.buf = make_depth(float("inf"), [(r0, r1, 160, 480, 0.50)])
    sent_after = []
    for tick in range(ps.REPORT_STREAK_N):
        bot._approach_victim()
        sent_after.append(len(robot.sup_emitter.sent))
    check("report withheld until REPORT_STREAK_N frames",
          sent_after[:-1] == [0] * (ps.REPORT_STREAK_N - 1) and sent_after[-1] == 1,
          "messages sent per tick: %s (streak N=%d)" % (sent_after, ps.REPORT_STREAK_N))
    check("opportunistic find does not retire a distant waypoint",
          bot.target_idx == 0,
          "target_idx=%d after reporting 19 m from waypoint (20.0, 20.0)"
          % bot.target_idx)

    # ── 8. A find AT the waypoint does retire it ────────────────────────────
    bot._my_waypoints = [(6.4, 6.2)]          # ~0.45 m from the robot
    bot.target_idx = 0
    bot._homing = True
    bot._home_start_time = robot.getTime()
    bot._report_streak = 0
    bot._reported_positions = []
    for _ in range(ps.REPORT_STREAK_N):
        bot._approach_victim()
    check("find at the waypoint retires it",
          bot.target_idx == 1,
          "target_idx=%d" % bot.target_idx)

    # ── 9. The emitted message still matches the channel-43 contract ────────
    import json
    msg = json.loads(robot.sup_emitter.sent[-1].decode())
    keys = {"timestamp", "robot_id", "position", "victim_found", "victim_confidence"}
    check("channel-43 message contract intact",
          set(msg) == keys and msg["victim_found"] is True
          and abs(msg["victim_confidence"] - 0.9) < 1e-9
          and len(msg["position"]) == 3,
          json.dumps(msg))

    bad = [r for r in results if not r[0]]
    print("\n%d/%d checks passed" % (len(results) - len(bad), len(results)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
