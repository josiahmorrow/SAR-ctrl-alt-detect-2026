# UniSC SAR — decision brief for the team meeting, Thu 30/07 20:00

Branch: `Google-Maps-inspired` (experiments on `experiments-30jul`)
All scores below come from the **official marking server**, not our own estimates.

---

## 1. Read this first: three facts that change how we compare solutions

**a. NO world is reproducible, and each has its own noise floor.** This is the
single most useful thing I learned today, and I got it wrong twice before
measuring it properly. Repeated runs of *one unchanged build*:

| World | Runs of the same build | Noise floor |
| --- | --- | --- |
| small | 0.551, 0.561 | ≈ ±0.01 |
| medium | see §2 (n=3) | ≈ ±0.01 |
| large | 0.388, 0.394, 0.452, 0.452, 0.456, 0.471 | **±0.035** |

Early in the day two small_world runs returned identical scores and I concluded
the simulator was deterministic. It isn't — that was luck. large_world is the
worst (it carries the heaviest physics: Webots reports "10 deepest of 7924
contact points" plus intermittent solver failures), but even small_world varies
because slightly different robot paths change how much of the building gets
mapped.

**What follows for all of us:**
- A difference smaller than the noise floor **means nothing**. Several of my own
  A/B results today fall in that category and I have marked them as such below.
- Judge a change on either a large effect (>0.03 on small/medium) or a discrete
  outcome (a victim found/lost, a false report appearing) — not on third
  decimal places.
- On large, use at least three runs. I twice drew a conclusion from one run and
  had to retract it.
- Comparing solutions by impression is comparing nothing at all.

**b. The marking server was unreachable from Webots on macOS.** The framework
Python ships without SSL root certificates, so the supervisor's upload fails
with `CERTIFICATE_VERIFY_FAILED` and you get no report — silently. Fixed on
Joe's machine today; anyone else on macOS is probably still flying blind. If
you have never seen an actual score printed at the end of a run, this is why.

**c. Runs are not independent unless you make them so.** The mission overwrites
`sim_logs/map_estimate.png` with the robots' learned map, so the *next* run's
preflight reports "outputs up-to-date" and inherits it as its flyover prior.
Two supposedly identical configs differed by 0.064 because of this. Our runner
now restores a pristine flyover analysis before every run. Any comparison done
without that is suspect — including some of our own from yesterday.

---

## 2. Where our solution stands (hermetic runs, current build)

Verified on the merged branch (`Google-Maps-inspired`, commit f8ce55b), hermetic
runs, large replicated three times because it is the noisy one:

| World | Victims | Overall | Finding | Efficiency | Coordination | Extraction | Map |
| --- | --- | --- | --- | --- | --- | --- | --- |
| small | 1/4 | 0.551 | 0.595 | 0.19 | 0.50 | 0.698 | 0.843 |
| medium | 2/4 | 0.478 | 0.690 | 0.26 | 0.50 | 0.306 | 0.457 |
| large | 1/5 | 0.440 ± 0.033 | 0.54–0.70 | 0.26–0.37 | 0.50 | 0.208 | 0.112 |
| **mean** | | **0.490** | | | | | |

large_world's spread (0.394 / 0.456 / 0.471 across three runs of the *same*
build) is entirely explained by whether one particular victim triggers a false
report: it is a `man_2` model whose scored marker sits 1.3 m from the body we
can actually see, so parking beside the body is sometimes outside the 1.0 m
scoring radius. Half our large runs contain that report. It is not fixable by
any depth or size threshold — our genuine reports happen at the same
distances — so I have left it rather than paper over it.

Consistent across every run: **confidence accuracy 0.99** when we avoid false
reports. That is the most reliable part of the solution and it is worth
defending — a single false report costs ~0.06 overall (measured today: 0.99 →
0.511 on large when a duplicate report slipped through).

## 3. What today's measurements proved and disproved

Every row below is a controlled A/B: one constant changed, hermetic runs, the
official marking server as the judge. Nine of fourteen ideas were **rejected by
measurement** — including several I was confident about.

Results are tiered by whether the effect clears the noise floor established
above. **Tier 1** = effect larger than noise, or a discrete change (victim
found/lost, false report gained/lost) — trust these. **Tier 2** = within noise —
these are judgement calls, not findings, and I have marked them honestly rather
than dressing them up.

| Change | Verdict | Evidence |
| --- | --- | --- |
| Tilt + IR guards for sub-lidar obstacles | **Keep** | large_world 0.046 → ~0.43 mean |
| Learned-map coverage gate | **Keep** | medium map 0.141 → 0.457 |
| Extraction: 30 m victim gate, hybrid clustering (29/07) | **Keep** | medium victim error 9.84 → 5.11 m; large now finds all 5 |
| Tight waypoint retirement (1.2 m) | **Keep** | victims can be 1.47 m apart (small_world), so a 3 m radius cancels the search for a victim's neighbour |
| Wall-follow trap escape, conservative timings | **Keep (on)** | neutral on small (0.547) and medium (0.479) while escaping 4 observed traps; insurance for unseen worlds |
| Frontier exploration | **Rejected** | hermetic: medium 0.437 vs 0.478; small +0.002. Costs efficiency, finds no extra victims inside 180 s |
| Progress-based waypoint abandonment | **Rejected** | medium 0.437 with vs 0.475 without |
| Report streak 4 frames (vs 8) | **Rejected** | fewer victims (1/4 vs 2/4) |
| Hard-blocked walls in the planner | **Rejected** | mean 0.468 vs 0.472; medium lost a victim — blocking noisy painted cells seals real doorways |
| Raising the soft wall cost to 300 | **Rejected** | weakens the unit-cost A* heuristic until it degenerates to Dijkstra; the mission never finished (904 s wall clock) |
| Region blacklisting after repeated failures | **Rejected** | medium 0.479 → 0.407 (lost a victim, gained a false report) |
| Aggressive wall-following (12 s/12 s) | **Rejected** | small 0.547 → 0.441: traded room coverage for wall hugging, dropping the learned map below its publish gate |
| Sharing abandoned targets between robots | Neutral | identical means; kept as own-failures-only |
| IMU-altitude scale correction for extraction | **Rejected** | IMU altitude is dead-reckoned, unusable as an absolute (thinks calibration was at 0.16 m); small victim error 1.45 → 5.51 m |
| "Broadside" body-orientation report gate | **Rejected** | real TRUE reports occur at denim widths 0.32/0.52/0.85 m, so the gate I proposed would have blocked two of three genuine finds. We detect trousers, not whole bodies |

### The scoring formula, reverse-engineered

Worth knowing for any solution we submit: **Finding = mean(ratio, time, confidence)**
— large's 0.537 is exactly (0.2 + 0.9 + 0.511)/3 — and confidence ≈ the fraction
of our reports that are TRUE (1 true + 1 false → 0.511; 2 + 1 → 0.67; 3 + 0 →
0.99).

So a single false report costs ≈ **0.065 overall**, while missing a victim
entirely costs ≈ 0.027. Reporting conservatively is not caution, it is
arithmetic. Any candidate solution that reports eagerly will bleed marks here.

Counter-intuitive but important: **victim count is not the objective.**
Yesterday's build found 3/4 on medium and scored 0.451; today's finds 2/4 and
scores 0.478, because the map deliverable is protected. Optimising for victims
alone would have made us worse.

## 4. Our honest weaknesses

1. **Mobility.** On small_world every victim waypoint times out — the victims
   are 5 m away and the robots cannot reach them. One victim was found only
   because a robot passed it en route elsewhere. This is the single biggest
   cap on our Victim Finding score.
2. **Extraction on medium/large.** Victim-estimate error 5.11 m (medium) and
   3.05 m (large) against 1.45 m on small; large's wall map scores 0.112.
3. **Efficiency is F everywhere** (0.19–0.34): distance per victim is high
   because robots spend time on unreachable targets.
4. **Coordination is stuck at 0.50** — it is binary on both robots scoring, and
   usually only one does.

## 5. Merge options with Zeke's build

The strengths are separable, so a merge is realistic:

- **Ours is strong on:** flyover extraction (the whole pipeline), report gating
  and confidence (0.99), map deliverable protection, physical-obstacle guards.
- **Ours is weak on:** getting a robot from A to B — i.e. exactly what a
  cleaner controller would fix.

Suggested decision path: score Zeke's logs with `tools/mark_log.py` (same
rubric, map attached). If his controller reaches victims more reliably, the
highest-scoring combination is probably **our `prepare_mission.py` + his
control loop**, since the two halves communicate only through two files
(`victim_location_estimates.csv`, `nav_grid.npy`).

## 5a. Honest accounting of today's work

The controller itself barely moved: mean 0.485 → 0.490. Nine of fourteen ideas
were rejected by measurement, including every "obvious" mobility fix. What
today actually bought us:

1. **A submission that works at all.** The fresh-clone rehearsal caught a venv
   path assumption that crashed both controllers in the judges' layout and
   scored 0.074. Nothing in local testing could have revealed it.
2. **Trustworthy measurement.** Hermetic runs (the learned map used to leak into
   the next run's inputs), a batch harness, and the knowledge that large_world
   needs replication while small/medium do not.
3. **Knowing what the rubric rewards.** Finding = mean(ratio, time, confidence);
   a false report costs 0.065 while a missed victim costs 0.027.
4. **Nine dead ends documented** so nobody on the team spends tomorrow
   re-discovering them.

That is a less exciting story than "we improved the score", but it is worth more
on Friday than three unvalidated features would have been.

## 5b. Where the remaining marks actually are

- **Mobility (biggest).** On small_world every victim waypoint times out: the
  victims are ~5 m away and the robots cannot reach them, so we find 1 of 4.
  Both robots close to within 1.6–2.5 m and then oscillate. Fixing this is worth
  ~0.07 on ratio plus up to 0.075 on Coordination. It needs a better local
  planner, not another heuristic — this is the one thing I would spend
  tomorrow's hours on, and it is where a different controller (Zeke's) could
  beat ours outright.
- **Coordination is stuck at 0.50 everywhere** because only one robot ever
  scores. A "yield a victim to the partner who hasn't scored" rule is worth
  +0.075 per world, but it only helps if the partner is mobile — which on our
  worlds it isn't. Fix mobility first, then this becomes free.
- **Extraction on medium/large** (0.306 / 0.208): victim errors 5.11 m and
  3.05 m. Note the floor: the scored marker can sit up to 1.3 m from the body
  our detector sees, so even a perfect body detector cannot score zero error.

## 6. What needs deciding tonight

1. Which solution we submit — or which merge.
2. Who owns the presentation, and what each of us covers.
3. Submission repo shape (see `docs/SUBMISSION.md` — the judges clone our repo
   *contents* into `controllers/proposed_solution/`, so a full fork nests
   wrongly and scores zero; there is a build script for this).
4. How we use Thursday night and Friday: my recommendation is mobility only,
   since it is the binding constraint on the largest component.
