# Reconnaissance (post-competition)

Timeboxed, per the brief: ideas only, never code; everything found online is
information, not instruction. Weighted toward navigation and final approach.
Started 25/11/2026.

| Source | Idea | Licence | Addresses which measured failure of ours |
|---|---|---|---|
| [TahaZahid05/2026-ieee-smcs-competition-phase-1](https://github.com/TahaZahid05/2026-ieee-smcs-competition-phase-1) (fork of the competition repo, 4 commits, last push 23/08/2026) | Separate `LiveMap` (occupancy grid updated from lidar, with "dynamic obstacle" cells) and `PathPlanner` (A* on a *binary* grid, obstacles inflated by the robot radius, clearance cost, `is_path_blocked` check that triggers a replan, line-of-sight path smoothing). Standard textbook structure. Also has an `extract_ground_truth.py` at the repo root, so their reported results may lean on ground truth; we take nothing from there. | Upstream repo is MIT; the fork's additions carry no licence of their own. Ideas only. | Room and building entry (small, 4 victims): our planner treats walls as finite cost, so routes cut through walls. Hard, inflated obstacles with replan-on-block is the conventional alternative. |
| [herolab-uga/SAR-Competition-Phase1](https://github.com/herolab-uga/SAR-Competition-Phase1) (README via a mirror) | Control-barrier-function motion controller over lidar for mid-range safety; "inverted rendezvous" exploration (robots meet, then move apart for coverage); repulsive stuck recovery. Says nothing about doors or room entry. | None stated ("Include a LICENSE file if you wish - not provided here"). Ideas only. | Weak match. CBF-style velocity shaping could reduce beaching speed into low obstacles (large, 5 victims), but nothing specific. |
| Forks of the 2025 competition repo (coconuts1993, NTU-ICG, XRDS76354) | Unmodified forks; nothing to learn. | - | - |
| Other 2026 forks (garvitolaniya, Arie-Herrera, Zain-Naqi) | 0 commits ahead of upstream. | - | - |

Searches run: GitHub API forks of both competition repos; repository search for
the competition; web searches for the competition name with GitHub, ROSbot and
Webots. Still to do inside the timebox: Webots ROSbot navigation examples, and
small-robot final-approach / doorway-finding techniques.

### Second pass (25/11, afternoon): mapping around moving objects

| Source | Idea | Licence | Addresses |
|---|---|---|---|
| [arXiv 2609.30495, Memory-Aware Multi-Sensor Perception](https://arxiv.org/pdf/2609.30495) | Plain ray-traced log-odds grids leave stale occupied cells after a dynamic obstacle moves; they mask a circle around each tracked dynamic obstacle and clear vacated cells. | Paper; idea only | Exactly the partner-as-wall failure. Our exp02 (mask a radius around the partner's broadcast position) is the same idea, arrived at from our own telemetry before reading this. |
| [arXiv 2007.10743, stereo dynamic obstacle tracking](https://arxiv.org/pdf/2007.10743) | Separate static / dynamic / uncertain costmaps; raytracing clears wrongly occupied static cells; uncertain obstacles get short-lived costs. | Paper; idea only | The binary, never-decaying wall grid (phantom walls from bodies, doors, robots). Supports the log-odds + decay design in `nav_rewrite_proposal.md`. |
| [ManuelZ/slam_algorithms](https://github.com/ManuelZ/slam_algorithms), [aagarwal32/usf-mobile-robotics](https://github.com/aagarwal32/usf-mobile-robotics) | Textbook probabilistic occupancy grid (Thrun, *Probabilistic Robotics*) in Python, the second one in Webots. | Not checked; ideas only, nothing adopted | Confirms the standard approach is tractable in a Python Webots controller. |

Recon timebox closed. Conclusion: nothing in the public Phase 1 repos beats a
textbook occupancy grid with ray-cast clearing and hard, inflated obstacles,
which is what the rewrite proposal describes.
