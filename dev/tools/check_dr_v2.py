"""Offline score of dev/tools/flyover_dr_v2.py against the world files.
Run from controllers/proposed_solution:  python -X utf8 dev/tools/check_dr_v2.py
Ground truth is used for scoring only."""
import os
import sys

import numpy as np

sys.path.insert(0, ".")
sys.path.insert(0, "dev/tools")
import eval_harness as eh            # noqa: E402
import flyover_dr_v2 as dr           # noqa: E402
import flyover_extraction as fx      # noqa: E402

REPO = os.path.abspath("../..")
for w in sys.argv[1:] or ("small", "medium", "large"):
    est = dr.locate_victims(os.path.join(REPO, "recordings", f"{w}_world_flyover.mp4"),
                            os.path.join(REPO, "recordings", f"{w}_world_flyover.csv"), fx)
    gt = eh.load_world(os.path.join(REPO, "worlds", f"{w}_world.wbt"))
    xy = np.array([[e[0], e[1]] for e in est]) if est else np.zeros((0, 2))
    print(eh.victim_report(xy, gt["victims"], gt["origin"])[0], flush=True)
