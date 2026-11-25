"""Build the time-curriculum development copies. Originals are never touched.

* controllers/sar_marking_supervisor_dev/sar_marking_supervisor_dev.py
    The organisers' supervisor with ONE line changed: the mission length
    default reads SAR_MISSION_S (default 180). Scoring logic is identical; the
    script prints the full diff and refuses to write if anything else differs.
* worlds/<world>_world_dev.wbt
    The organisers' world with ONE line changed: the supervisor node's
    controller field names the dev supervisor.

Results count toward the goal only at 180 s on the untouched originals
(run_batch.py uses those whenever --mission-s is 180). Both outputs are
excluded from the organisers' repo via .git/info/exclude.

    python dev/tools/make_dev_supervisor.py
"""
import difflib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_pristine import REPO                 # noqa: E402

SUP = os.path.join(REPO, "controllers", "sar_marking_supervisor",
                   "sar_marking_supervisor.py")
DEV_DIR = os.path.join(REPO, "controllers", "sar_marking_supervisor_dev")
DEV_SUP = os.path.join(DEV_DIR, "sar_marking_supervisor_dev.py")
OLD = "        max_duration: int = 180,\n"
NEW = ('        max_duration: int = int(os.environ.get("SAR_MISSION_S", "180")),'
       '  # DEV COPY: only change\n')
WORLD_OLD = '  controller "sar_marking_supervisor"\n'
WORLD_NEW = '  controller "sar_marking_supervisor_dev"\n'
WORLDS = ("small", "medium", "large")


def one_line_copy(src_path, old, new, dst_path):
    raw = open(src_path, "rb").read()
    crlf = b"\r\n" in raw
    text = raw.decode("utf-8").replace("\r\n", "\n")
    if text.count(old) != 1:
        sys.exit(f"!! expected exactly one {old.strip()!r} in {src_path}")
    out = text.replace(old, new)
    diff = [l for l in difflib.unified_diff(text.splitlines(True),
                                            out.splitlines(True), n=0)
            if l.startswith(("+", "-")) and not l.startswith(("+++", "---"))]
    if len(diff) != 2:
        sys.exit(f"!! diff for {dst_path} is not exactly one line: {diff}")
    if crlf:
        out = out.replace("\n", "\r\n")
    os.makedirs(os.path.dirname(dst_path), exist_ok=True)
    open(dst_path, "wb").write(out.encode("utf-8"))
    print(f"{os.path.relpath(dst_path, REPO)}:")
    for l in diff:
        print("   " + l.rstrip())


def ensure_excluded():
    ex = os.path.join(REPO, ".git", "info", "exclude")
    want = ["/controllers/sar_marking_supervisor_dev/"] + \
        [f"/worlds/{w}_world_dev.wbt" for w in WORLDS] + \
        [f"/worlds/.{w}_world_dev.*" for w in WORLDS]
    have = open(ex).read().splitlines() if os.path.exists(ex) else []
    add = [w for w in want if w not in have]
    if add:
        with open(ex, "a") as f:
            f.write("\n" + "\n".join(add) + "\n")


if __name__ == "__main__":
    src = open(SUP, encoding="utf-8").read()
    if "import os" not in src:
        sys.exit("!! supervisor no longer imports os; review the dev patch")
    one_line_copy(SUP, OLD, NEW, DEV_SUP)
    for w in WORLDS:
        one_line_copy(os.path.join(REPO, "worlds", f"{w}_world.wbt"),
                      WORLD_OLD, WORLD_NEW,
                      os.path.join(REPO, "worlds", f"{w}_world_dev.wbt"))
    ensure_excluded()
