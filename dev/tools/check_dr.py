import sys, os, numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "dev/tools")
import flyover_extraction as fx, flyover_dr as dr, eval_harness as eh
REPO = os.path.abspath("../..")
for w in ("small", "medium", "large"):
    est = dr.locate_victims(os.path.join(REPO, "recordings", f"{w}_world_flyover.mp4"), os.path.join(REPO, "recordings", f"{w}_world_flyover.csv"), fx)
    gt = eh.load_world(os.path.join(REPO, "worlds", f"{w}_world.wbt"))
    print(eh.victim_report(np.array([[e[0], e[1]] for e in est]), gt["victims"], gt["origin"])[0])
