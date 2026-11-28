# Resume point (updated 27/11/2026 04:15 AEST)

Harness: parN.ps1 over sar-env/2/3; log sar-env\_eval\q_parN.log; chained
batches via after_par.ps1 (waits for q_parN.done). agg.py / beach.py /
falses.py / repfeat.py in _scratch (run the last three from ...\dev\eval\runs).
CAUTION: killing by command-line match also matches your own shell if its
command mentions parN.ps1 - exclude $PID.

Kept stack: exp24 = exp23 + exp24 = ps_kept.py = committed
proposed_solution.py (0bfa437). exp24: small 2.67 (0.17 F), medium 2.83 (0.83 F).
(exp23 = exp08 + exp16 + exp19 + exp20: 2.67 / 2.67.) nav4 is built on exp23;
fold exp24 in with make_exp24.py if nav4 is kept.

PLANNER REPLACEMENT APPROVED by Joe 27/11 04:03 ("Go").
nav4 = ps_kept + make_nav1.py (log-odds map, hard chassis-inflated A*,
free-goal) + make_nav4.py (unknown cost 1.5, route re-validation every
0.5 s for hard routes, Bug2 off, heuristic weight 1.3). Offline test:
test_nav4.py (door, narrow door, unknown preference, goal in body, re-check,
timing <= 0.2 s).

nav4 result: small 1.67, medium 2.33 - not kept (sparse startup map:
69-86 wall cells vs ~1000; robot left the building on an unseen route).
nav4b (envelope mask) not run: small bbox reaches x 12.85, useless.
CORRECTION: "sparse startup map" was wrong (old map paints 3x3 per hit).
nav4c: small 1.40, medium 2.75 - rejected. New planner hurts small (route
choice in furnished rooms), neutral on medium.
Running: jobs12 ablation on ps_kept (exp24), 3 seeds: nav4d (log-odds map
only) and nav4e (binary map + hard planner + re-check, unknown 1.0, WF on),
~07:35-08:55 (make_ablation.py).
traj.py <run_dir> <robotN> [every] [t0] [t1] = condensed per-robot timeline.
Last commit: c992446.

STATUS DOC for Joe (Claude Docs): https://claude.ai/artifact/WKhaM2CV27KNVa2WZ7mHCK
(doc id ed756a07-bf42-4557-bf62-5dbab9e272d0, body node 0e85b8ad-4c75).
Update it at every check: At a glance table, Experiment log (newest first),
Where victims are lost, Current plan, and add a row to "Updates to this doc".

27/11 09:05: planner ablation negative (nav4d 1.67/2.00, nav4e 1.33/1.33 at
3 seeds vs exp24 2.67/3.33). PLANNER LINE CLOSED. Streak since exp24: 4.
Running: jobs13 exp24x = ps_kept seeds 6-11 (S+M), ~09:00-10:20.
Last commit: e1396bf. Status doc updated to rev 21.
Asked Joe: continue local fixes / new nav approach / stop and write up?

27/11 10:30: exp24x (kept stack seeds 6-11) lower: small 2.17, medium 2.50
(4 seeds). 12-seed kept estimate ~2.4 / ~2.7. Scorer uses TRUE robot
position (read only). Queued (chained, jobs14): exp25 = kept + exp18 flank
triggered by denim depth spread p90-p10 >= 0.35 m (make_exp25.py, uses
make_exp18k.py). Compare against exp24 seeds 0-5. Streak 4. Commit f9d492f.
Status doc rev 23.

27/11 12:10: exp25 rejected (small 2.83, medium 2.33). Streak 5. Commit
cbb62b3. Status doc rev 29. Running jobs15: v180 = ps_kept at 180 s on the
UNTOUCHED worlds (parN -MissionS 180), small/medium/large seeds 0-2.

27/11 13:00: v180 (kept @180 s, untouched): small 2.33, medium 2.33, large 0/5.
missed.py: small v1 seen but blocked near-prior-report (9/12 misses);
small v4 rejected denim-area-small (0.026-0.031 m2); medium v1 never seen.
Running jobs16: exp26 = area gate 0.022 within 3 m of unreported estimate
(make_exp26.py), ~12:55-14:15. Commit 8afa202. Status doc rev 31.

27/11 14:25: exp26 seeds 0-5 mixed (small 2.67, medium 2.33). Running
jobs17 exp26x seeds 6-11 (to ~15:40); decide on 12 seeds vs exp24+exp24x
(small 2.42, medium 2.67). Next idea for small v1: detect two distinct
denim bodies in one frame -> licence a second report near a prior one;
first do a snapshot run (SAR_SNAP_DIR, small seed 6) to inspect frames.
Commit a784939. Status doc rev 34. Streak 5 (exp26 pending).

27/11 15:45: exp26 REJECTED on 12 seeds (medium 2.30 vs 2.67). Streak 6.
Commit c3f607f. Status doc rev 41. snaprun.ps1 launched: waits for exp26x
S11/M11, then kept stack small seed 6 with SAR_SNAP_DIR =
sar-env\_eval\snaps\snapS6 (tag snapS6). Inspect robot2's frames after it
reports v2 (~31 s): are v1 and v2 both visible as separate denim blobs?

27/11 16:45: snapS6 done. v1/v2 never two blobs in one frame. Real cause:
REPO_MATCH disclaims the v1 estimate (2.94,-3.12) at the v2 report, then
homing advances the itinerary anyway and HOME_BLOCK_R_M (1.5) blocks v1
(147 frames). Same disclaimer in 5/6 exp24 small runs.
exp27 = make_exp27.py on ps_kept -> ps_exp27.py (disclaimed-estimate
licence, TWO_BODY flag). Running jobs18 (S+M seeds 0-5) from 16:33, ~17:55.
Decide: kept if both worlds up/level (mixed -> extend seeds 6-11 per
protocol). grep logs for "exp27 two-body report" / "exp27 keeping".
Commit f0e5a35. Status doc rev 43. Streak 6.

27/11 18:20: exp27 REJECTED (small 1.40 /5, medium 2.20 /5, S2 void
timeout). Reverted (main env = ps_kept, clean). Commit dacaa46. Doc rev 46.
Streak 7. Over 24 kept runs: end-on near misses (report 1.0-1.2 m) cost 5
victims (small S7 v3, S10 v2; medium M5/M6/M11 v3 at ~(13.2,-20.6), the v3
flyover estimate spot); crate phantom at ~(15.4,-12.6) in 8/12 medium runs.
Camera depth bottoms at 0.62 m so "report closer" is not available.
Put to Joe 18:20: (a) switch to 180 s-focused work, (b) generalisation
check, (c) stop and write up, (d) spend last 3 experiments on end-on
flanking / phantom. No batch running. Next check 21:00.

27/11 20:15: Joe chose (d) at 19:31. End-on near misses: no separating
feature, no experiment. snapM3 (medium s3, crate at 73 s) + texcheck.py:
hp = mean|V-blur5(V)|/meanV on eroded denim blob; crate 0.0047-0.0070 at
report range (0.0094 at 1.1 m), victims >= 0.0103. exp28 = make_exp28.py on
ps_kept (FINE_HP_MIN 0.0085 at report time + flat-spot homing block 1.0 m).
jobs19 running from 20:12 (~21:35). grep "exp28 flat surface". Kept only if
crate phantom gone and no true victim refused / finds not down. Commit
1c4e6cb. Doc rev 47. Streak 7. The 21:00 scheduled check predates this:
just confirm exp28 is running.

27/11 21:50: exp28 REJECTED. Crate refused in all medium runs, but medium
v3 (11.9,-20.4) has hp 0.0044-0.0077 (same as crate) and was refused
(found 0/6). Small 2.83, medium 2.33. Commit 39cae23. Doc rev 49. Streak 8.
No sound hypothesis left for option (d). Mini idle, main env = ps_kept.
Waiting on Joe: (b) generalisation check, (c) write up, or (a) 180 s work.
Large world: recommend accept as blocked under rule 2.

28/11 08:15: JOE DIRECTED: fix large (large_world_fix.wbt = no floor(2), dev supervisor; run_batch default, SAR_LARGE_FIX=0 for untouched), work at 180 s (parN -MissionS 180), no stop rule, target 90% all worlds. jobs20 b180 = ps_kept seeds 0-5 S/M/L at 180 running from 08:12. Then: missed.py/falses.py per world at 180, pick fixes by loss size. Ideas: geofence return via trail (if exits <180 s), large-specific losses, time budget (waypoint timeouts, scan durations) at 180 s.

28/11 08:20: Joe: push every idea; relax zero-false and run generalisation check if they help. Chained after b180: jobs21 r26 = ps_exp26 (smaller denim gate near estimates) at 180 s S/M/L seeds 0-5 via after_par180.ps1 (~09:00-09:50).

28/11 09:10: b180 interim: S 2.40, M 2.20, L(repaired) 2.00/5; loss table in log. Queue: r26 (after_par180, auto after b180) then exp29 = make_exp29.py (reported-body homing block 1.2 m) via chain2.ps1 -After r26 (jobs22). Next ideas: partner fp-zone only when partner is in camera FOV (medium v2 M0: both robots blocked at 0.7 m); large v3/v5 never approached (4 estimates for 5 victims, exploration too slow / robots loop at reported body); medium v1 no candidate at 1.2-2 m. NOTE: in PS loops do not use $s as loop var with $S path (case-insensitive clash).

28/11 10:55: b180 FINAL S 2.50 / M 2.17 (0.5F) / L 1.83 of 5. r26 rejected. exp29 running (till ~11:30), then jobs23 via chain2 -After exp29: exp30 (no back-to-back waypoint retry) and exp31 (partner fp-zone only in FOV), each on ps_kept, ~11:30-13:15. Then combine winners (apply make scripts in sequence on ps_kept) and run combo; generalisation check after a keep.

28/11 13:35: exp29, exp30 REJECTED at 180 s. exp31 running (ends ~14:10), then chain_mac.ps1 runs macL = ps_kept on large-repaired 180 s seeds 0-5 with SAR_PRISTINE_DIR=_eval\pristine_macwp (macOS estimates, err 0.13-0.96 m) = ceiling diagnostic (~14:10-14:40). KEY INSIGHT: main loss is flyover victim estimates (flyover_extraction.py IMU projection w/ duration-fitted constants; no estimate within 3 m for small v4, medium v1+v3, large v3+v5). Next thrust: physically grounded victim localisation from the flyover (ArUco origin marker gives scale; chain frame-to-frame homographies / visual odometry on the ground plane, project denim detections). New module, evaluate offline vs ground truth with eval_harness.py / compare_extraction.py before any sim run.

28/11 15:30: macL = 3.17/5 on large (vs 1.83) -> estimates ARE the lever. exp31 mixed; jobs25 running: b180x + exp31x seeds 6-11 S/M/L at 180 (till ~17:10). dev/tools/flyover_vo.py prototype: 2-D tracking (similarity/--rigid/--floor) fails from parallax (camera ~4 m up, m/px 0.0044 at 1920 wide; walls 2.4 m). --mosaic writes %TEMP%\mosaic_<world>.png with truth crosses. NEXT: essential-matrix VO (intrinsics from FOV; floor = deepest plane fixes scale from height; height from ArUco at takeoff), better victim detector (denim + skin + shape). Evaluate offline on all 3 worlds; target every victim within 1 m.

28/11 16:20: BREAKTHROUGH offline: dev/tools/flyover_dr.py (run with --flat; --mosaic writes %TEMP%\dr_mosaic_<w>.png) = body-frame dead reckoning (v=int acc_x, yaw=int gyro_z), ArUco anchor, marker handedness fix (swap X/Y if det>0). All 13 victims within 1.07 m (small .79 .07 .20 .38; medium .47 1.07 .15 .54; large .25 .59 .27 .91 .57). TODO: (1) victim detector to kill spurious blue objects (small 2, medium 1, large 5): denim + adjacent skin/clothing, shape; (2) height/scale without drift (floor flow vs IMU speed); (3) wire into preflight: replace victim CSV + waypoint meta source in the hybrid (flyover_extraction.detect_victims_with_imu_projection / analyse; keep file formats), regenerate pristine (make_pristine --force), run b180-style batch S/M/L 180 s seeds 0-5 as 'dr1'; (4) wall map later. b180x/exp31x (seeds 6-11) still running until ~17:10.

28/11 17:10: flyover_dr.py (solution module) + make_dr1.py -> ps_dr1.py. Pristine rebuilding with ps_dr1 (-X utf8 REQUIRED, else fx print of a tick char throws and it falls back); chain_dr1.ps1 then runs jobs26 dr1 (S/M/L seeds 0-5, 180 s). q_parN.log gets a '[chain] pristine rebuilt' line with the large CSV. Old cache in _eval\pristine_old. NOTE main-env proposed_solution.py currently = ps_dr1; ps_kept.py unchanged until dr1 is judged. If kept: ps_kept = ps_dr1, commit proposed_solution.py + flyover_dr.py, then re-test exp31 and exp29/exp30 on top, generalisation check, 12 seeds.

28/11 17:25: dr1 = WORSE (S 1.83, M 1.67, L 1.00) though estimates accurate: spurious estimates (beds, cart, forklift) eat robot time (second looks, timeouts). jobs27 running: dr2 (dr1 + SECOND_LOOK False + exp30) then dr3 (dr1 + SECOND_LOOK False), 36 runs till ~19:00. Pristine = new estimates (old in _eval\pristine_old; to go back to b180 conditions set SAR_PRISTINE_DIR to it). Do not edit flyover_dr.py while runs are queued (pristine hash -> rebuild). Ideas next: (1) arrive-and-dismiss fast: if within 2 m of an estimate with clear lidar line of sight and no denim candidate in a 360 scan, drop it at once; (2) estimate inside an obstacle (lidar says occupied) -> scan from the nearest free point, no stall wait; (3) order waypoints so each robot's nearest REAL-looking ones go first? no discriminator; (4) spread: assign by travel cost so one robot isn't given both forklift points; (5) beaching on large (robot2 tilt 10 from 86 s, odometry runaway).

28/11 19:10: dr2/dr3 batch FAILED instantly (PowerShell Set-Content -Encoding utf8 wrote a BOM into ps_dr3.py; make_pristine ast.parse choked). BOMs stripped. LESSON: never write .py with PS Set-Content utf8; use [IO.File]::WriteAllText or python. chain_dr23.ps1 now rebuilds pristine (utf8) then runs jobs27 (dr2 then dr3), ~19:25 to ~21:00.

28/11 21:20: dr2 2.00/1.60/1.33, dr3 2.00/1.75/1.00 (< b180 2.50/2.17/1.83). chain_cln.ps1 runs jobs28 'cln' = ps_dr3 + _eval\pristine_clean (spurious rows hand-removed; DIAGNOSTIC ONLY) after dr3 ends, ~21:20-22:10. Decision after cln: if cln >> b180 -> worth solving spurious (ideas: smear suppression near dropped elevated clusters; robot-side instant dismissal); if not -> controller time use is the limit (slow approach, adjacency block, beaching), report to Joe honestly.

28/11 22:30: cln: small 1.80 (worse than b180 2.50!), large 2.50 (better than 1.83), medium INVALID (controller re-ran preflight in-run and regenerated full CSV; check why outputs_are_fresh fails for medium under SAR_PRISTINE_DIR). Small losses with clean accurate estimates: v4 robot within 1.0 m 100+ s with 0 candidates; v1 blocked near-prior-report. NEXT (dr4 on ps_dr3): estimate-based reporting: (a) each report consumes its nearest estimate; homing/report near a prior report allowed when the candidate projects nearer to a different, unconsumed estimate (replaces HOME_BLOCK_R_M logic when estimates exist); (b) arrival report: within ~0.7 m of an unconsumed estimate and scan shows nothing -> report on position (flag: false reports at bedding/forklift). Test (a) and (a+b) separately at 180 s. Joe checkpoint sent 22:30.

28/11 22:45: make_dr4.py (+ --arrival = dr5) on ps_dr3 -> ps_dr4.py / ps_dr5.py. jobs29 via chain_dr45.ps1 (dr4 then dr5, 36 runs, ~22:35-00:20). grep logs for 'dr4 licensed report', 'dr4 estimate', 'dr5 arrival report'. Baseline b180 2.50/2.17/1.83; dr3 2.00/1.60/1.00. AFTER: flyover wall+obstacle map from flyover_dr poses into routing (Joe's question 22:22), then robot coordination (share map, claim targets, share body position).

28/11 22:55 JOE: foundation first. Tomorrow (29/11): make the flyover extraction produce correct outputs: victims CSV (done bar extras), WALL map and OBSTACLE map at fine grain (current grids are 0.05 m/px, 600x600), from flyover_dr poses; score offline vs truth; then feed the map to the robots' planner so they navigate the layout from the start. Order: (1) flyover_map.py offline wall map + score, (2) obstacle/height layer via parallax, (3) report numbers to Joe, (4) wire into nav grid, (5) then coordination. Controller tweaks (dr4/dr5) are secondary to this.

28/11 23:05 JOE: parallel agents approved; Brave closed (other browsers may be closed); Java and WSL/vmmem NOT to be touched (unsure what they are; vmmem may be Claude's VM). Paging file not approved. Stay at 3 sim slots (commit headroom ~3 GB). Usage meter 22:51: weekly all-models 67%, resets in ~1 day (Monday). THREE background agents launched 23:05, offline only, no git, own files: A = dev/tools/flyover_map.py (wall + obstacle map, outputs _scratch\agentA); C = dev/tools/flyover_dr_v2.py + check_dr_v2.py (drop false victim locations, _scratch\agentC); D = _scratch\make_co1..4.py + ps_co*.py (robot coordination patches on ps_dr3). Main session: commits their files, runs sims (dr4/dr5 now; then co1.. on best stack), keeps 3 slots busy.

28/11 23:20: coordination agent DONE: make_co1..4.py (+ ps_co1..4.py on dr3, ps_co*_dr4.py on dr4, ps_co1234*.py), compile-checked only. co1 = target claims (+ fp-zone deadlock relief; set CO1_FP_RELIEF False if stacking exp31), co2 = share body position in 'victim' msg (0.8 m), co3 = share sightings, co4 = share wall cells (riskiest). Robot-to-robot channel is 73. chain_co.ps1 queues jobs30 (co2 then co1 on dr3 base, S/M/L seeds 0-5, 180 s) after dr5 ends (~00:20 -> ~02:10). grep: 'co1 waypoint', 'partner-claim', 'partner-body', 'co2 report withheld'. Agents A (map) and C (false locations) still running.

28/11 23:35: map agent A DONE: dev/tools/flyover_map.py (sparse feature parallax). Walls correctly placed but low recall; best-shift IoU new(height)/new(shape)/current: small .242/.094/.271, medium .137/.241/.158, large .192/.211/.296 -> NOT good enough to replace graded map. Centring ~1 m off on small/medium. Raised-obstacle layer plausible, unvalidated; suggested use: soft cost prior in _nav_low via sim_logs\flyover_raised.npy. Second agent A2 launched: dense near-nadir orthomosaic approach, dev/tools/flyover_ortho.py, outputs _scratch\agentA2. Agent C (false locations) still running.

28/11 23:55: agent C DONE: dev/tools/flyover_dr_v2.py keeps all 13 victims, ZERO false locations offline (small .44 .07 .20 .38; medium .47 1.07 .15 .54; large .26 .60 .21 .91 .35). Rules: object > 1.0 m2 of connected blue-green = not a person; > 0.6 m2 other blue in crop = fabric; big-object footprint vote; limb thickness < 0.08 m = edge (FRAGILE: cart .072 vs thinnest victim .093). Thresholds tuned on these recordings (flag rule 7). chain_co.ps1 CANCELLED. chain_v2.ps1: after dr5 ends -> flyover_dr.py := v2 (v1 backed up to _scratch\flyover_dr_v1_backup.py, old cache to _eval\pristine_dr_v1), rebuild pristine, run jobs31: v2b (ps_dr3) then v2d4 (ps_dr4), S/M/L seeds 0-5 180 s (~00:35-02:30). q_parN.log gets '[chain] v2 pristine <world>' lines: expect 4/4/5 rows. Then co2/co1 (jobs30) on the best base. Agent A2 (ortho wall map) running.

29/11 00:15: agent A2 DONE: dev/tools/flyover_ortho.py. Best-shift IoU new/current: S .278/.271, M .183/.158, L .118/.296 -> not a replacement for graded map. Orthomosaics clean; prior files _scratch\agentA2\<world>_prior.npz (unseen/floor/wall/raised) usable as SOFT nav cost. Large walls 0.2-0.5 m inward (possible ~2% scale/drift at range: CHECK flyover_dr scale far from origin; medium v2 1.07 m error may be the same cause). No agents running now. Sim queue: dr5 ending, then chain_v2 (flyover_dr v2 + rebuild + v2b, v2d4).

29/11 00:30: dr4 1.67/1.50/1.60, dr5 partial 2.00/1.00/1.00 -> both rejected (dr4 licence fired 3x, all dup of v2 via the bedding 'v1' estimate; dr5 arrival never fired). chain_v2 will swap in flyover_dr v2 after dr5 ends (~00:50), rebuild, run v2b (dr3) + v2d4 (dr4) till ~03:00. Next check 03:05: judge v2b/v2d4 vs b180 (2.50/2.17/1.83); then queue co2/co1 on best base (ps_co*_dr4.py exist), 12-seed confirmation of any winner, soft map prior (agentA2 prior npz) as later experiment.

29/11 03:20: v2b 1.50/1.33/2.33, v2d4 1.60/1.20/2.75 (large > b180 1.83; small/medium < b180). flyover_dr.py in the solution folder is NOW v2 (uncommitted in git as flyover_dr.py; committed as dev/tools/flyover_dr_v2.py). Cause on medium: flat 40 s waypoint cap kills far waypoints (23 m) + STALL_ABANDON False. chain_to1.ps1 runs jobs32 after v2d4: to1 (make_to1.py: distance-scaled cap) then to2 (to1 + STALL_ABANDON True), base ps_dr3 + v2 estimates, till ~05:20. Next: best of to1/to2 + co2/co1; small v1 adjacency (dr4 licence had 1 dup on v2 estimates), soft map prior for trapped robots.

29/11 05:40: to1 1.67/1.00/2.50, to2 1.80/1.20/2.25, v2d4 final 1.50/1.33/2.83. Medium: robots reach 1.3-2.3 m of v1/v3 but camera has no candidate; ARRIVE_VIDEO_M 1.6 stops them short. chain_ar1.ps1 runs jobs33 after to2: ar1 = ps_dr5 with ARRIVE_VIDEO_M 0.6 (arrival report within 0.7 m of unconsumed estimate), then ar1co2; till ~07:35. grep 'dr5 arrival report'. BEST SO FAR: small/medium = exp24 + old estimates (2.50/2.17); large = v2 estimates + dr4 (2.83). Nothing beats b180 on all three. Morning summary sent 05:40.
