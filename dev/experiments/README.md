# Experiment scripts (post-comp)

Each `make_<name>.py <in> <out>` applies ONE experiment as a patch to a controller file and writes a new one; they are how every variant in `dev/docs/post_comp_log.md` was built. The generated controllers (`ps_*.py`) are not committed: rebuild them by applying the scripts in the order the log gives.

- Kept controller (exp24) = the committed `proposed_solution.py`.
- 180 s stacks: `make_dr1.py` (flyover_dr hook) -> SECOND_LOOK False (dr3) -> `make_dr4.py` [--arrival] -> `make_to1.py`, `make_co1..4.py` (robot coordination).
- Analysis: `agg.py` (means per world), `falses.py` (report verdicts), `missed.py` (why a victim was missed), `traj.py` (robot timeline), `beach.py` (tilt / stuck).
- Harness: `parN.ps1` runs a jobs file over three environment copies.
- `RESUME.md`: running notes of state and next steps.
