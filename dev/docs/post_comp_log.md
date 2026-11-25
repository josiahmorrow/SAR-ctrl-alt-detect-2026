# Post-competition log

Platform for every run below unless stated: Minisforum UM773 Lite, Windows 11
Pro, Python 3.14.5, numpy 2.4.6, opencv-contrib 4.13.0.92, Webots R2025a,
judges' layout (organisers' repo at `3fe3ca8`, supervisor and worlds
untouched), 180 s mission, hermetic runs via `dev/tools/run_batch.py`.

## Stage 1 - baseline (25/11/2026)

Controller code: `phase1-submitted` (`c6951a7`), unchanged.

| World | Runs | Victims found | False reports | Overall | Finding | Effic. | Coord. | Extraction | Victim est. err |
|---|---|---|---|---|---|---|---|---|---|
| small | 1 | 0/4 | 0 | 0.193 | 0.000 | 0.00 | 0.00 | 0.642 | 0.66 m |
| medium | 1 | 2/4 (t = 36.9, 145.7 s) | 1 | 0.466 | 0.554 | 0.23 | 0.50 | 0.451 | 0.95 m |
| large | 5 | 0/5 every run | 0 | 0.141 x5 | 0.000 | 0.00 | 0.00 | 0.469 | 4.95 m |

Mean of world means: **0.267**, against the Phase 1 record of ~0.55
(0.483-0.505 / 0.580 / 0.635-0.646, macOS). **Baseline does not reproduce.**

Large was identical in all five runs (0.141, same extraction, same outcome):
on this machine large behaves as deterministic, unlike the macOS record.

### Why: the flyover extraction gives different answers on Windows

Isolating run `iso-macwp` (large, 1 run): the same Windows build, but with
`victim_location_estimates.csv`, `victim_waypoints_meta.json` and
`map_estimate.png` replaced by the macOS hybrid outputs committed in the team
repo at `48ee484` (the only world for which they survive).

| large_world | Victim est. err | Victims | Overall |
|---|---|---|---|
| Windows preflight | 4.95 m | 0/5 | 0.141 |
| macOS preflight outputs, Windows mission | (Phase 1: 0.30-0.61 m) | 1/5 | **0.642** |
| Phase 1 record (macOS) | 0.30-0.61 m | 1/5 | 0.635-0.646 |

With the macOS waypoints the Windows mission reproduces the Phase 1 large
score to within 0.004. Controller, odometry and physics reproduce; **the
divergence is entirely in the preflight extraction.**

Waypoints vs ground truth, large (mat frame):

| GT victim | Nearest macOS estimate | Nearest Windows estimate |
|---|---|---|
| (8.77, 2.67) | (8.72, 2.10) 0.57 m | (11.89, 2.08) 3.17 m |
| (5.49, -5.76) | (4.71, -5.90) 0.79 m | (6.33, -6.15) 0.93 m |
| (15.92, -7.08) | (16.04, -7.14) 0.13 m | none within 4 m |
| (4.34, 2.74) | (5.06, 2.10) 0.96 m | (5.47, 6.01) 3.46 m |
| (13.10, -1.13) | (13.04, -1.71) 0.58 m | (11.89, 2.08) 3.43 m |

On Windows, `flyover_extraction` falls back to "IMU-assisted victim
estimates" on all three worlds (small: 1 confirmed image detection of 14 raw;
medium: 0 of 5; large: 6 of 51, still IMU-assisted). `prepare_mission` on
medium reports only 27% of samples posed and drops to IMU dead reckoning.

Ruled out so far:
* frame rate / duration: OpenCV FFMPEG backend reports 62.5 fps and the IMU CSV
  has exactly one row per frame (small 4801 rows over 76.8 s), so t = fi/fps is
  correct;
* seeking: `dev/tools/seek_check.py` shows `CAP_PROP_POS_FRAMES` is frame-exact
  on this build at six probe points;
* world files: `large_world.wbt` is byte-identical (same blob) to the team copy;
  the earlier "392 insertions" diff was CRLF line endings.

Leading hypothesis, not yet tested: decoded pixel values differ between the
macOS (AVFoundation) and Windows (FFmpeg) OpenCV backends (YUV to RGB matrix or
range), moving colours across the HSV thresholds both detectors use. This is
also a generalisation weakness in its own right: the judges' platform was never
known.

### Harness changes made to get here (no controller change)

* `run_batch.py` ported to Windows; per-run supervisor JSON and marked report
  copied beside the log; CSV now carries ground-truth victims found, the sim
  time of each find, reports and false reports.
* `telemetry/sitecustomize.py`: Webots on Windows drops `--stdout`, so the dev
  venv tees controller and supervisor output to files when `SAR_TEE_DIR` is set.
* `warm_webots_cache.py`: Webots' own asset download stalled on medium (17
  assets missing) and large (1), hanging world load with no controller started.
  The batch now pre-fetches each world's asset closure and has a 150 s load
  watchdog with retry.
* `make_pristine.py`: per-world flyover snapshot produced by the controller's own
  `_analyse_both` (lifted with `ast`), replacing the old cache that held only
  our own extraction, not the shipped hybrid.
