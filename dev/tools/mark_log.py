"""
Officially re-mark a saved SupervisorLog against the competition marking
server, replicating the supervisor's multipart/form-data send — INCLUDING the
map images the README's JSON-only example omits (without them the server
reports "Missing map estimate image" and scores the map 0.000).

    .venv/bin/python tools/mark_log.py log/SupervisorLog_<...>.json worlds/small_world.wbt
    .venv/bin/python tools/mark_log.py log/SupervisorLog_<...>.json worlds/small_world.wbt \
        --map controllers/proposed_solution/sim_logs/map_estimate.png

The estimate map defaults to the current sim_logs/map_estimate.png — make sure
it corresponds to the same world as the log (preflight regenerates it per
world).  Ground truth is rendered from the .wbt via eval_harness (replicates
the supervisor's generator).  Dev tool only — never part of the submission.
"""
import argparse
import json
import os
import ssl
import sys
import uuid
from io import BytesIO
from urllib import request as urllib_request

import numpy as np
from PIL import Image


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
sys.path.insert(0, HERE)

import eval_harness as EH  # noqa: E402

URL = "https://ieee-sar-hackathon.ts.r.appspot.com/mark-log"


def multipart(parts_text, parts_file):
    boundary = f"----SARBoundary{uuid.uuid4().hex}"
    out = []
    for name, value in parts_text:
        out.append((f"--{boundary}\r\n"
                    f'Content-Disposition: form-data; name="{name}"\r\n\r\n'
                    f"{value}\r\n").encode())
    for name, fname, ctype, data in parts_file:
        out.append((f"--{boundary}\r\n"
                    f'Content-Disposition: form-data; name="{name}"; '
                    f'filename="{fname}"\r\n'
                    f"Content-Type: {ctype}\r\n\r\n").encode() + data + b"\r\n")
    body = b"".join(out) + f"--{boundary}--\r\n".encode()
    return body, f"multipart/form-data; boundary={boundary}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("world")
    ap.add_argument("--map", default=os.path.join(
        REPO, "controllers", "proposed_solution", "sim_logs",
        "map_estimate.png"))
    args = ap.parse_args()

    with open(args.log) as f:
        log = json.load(f)

    files = []
    if os.path.exists(args.map):
        with open(args.map, "rb") as f:
            files.append(("map_estimate_image", "map_estimate.png",
                          "image/png", f.read()))
    else:
        print(f"[mark] WARNING: no estimate map at {args.map}")

    world = EH.load_world(os.path.join(REPO, args.world))
    gt_arr, _meta = EH.generate_ground_truth_map(world["walls"])
    buf = BytesIO()
    Image.fromarray(np.asarray(gt_arr, dtype=np.uint8)).save(buf, format="PNG")
    files.append(("map_ground_truth_image", "ground_truth_map.png",
                  "image/png", buf.getvalue()))

    body, ctype = multipart([("sar_log", json.dumps(log))], files)
    req = urllib_request.Request(URL, data=body,
                                 headers={"Content-Type": ctype},
                                 method="POST")
    ctx = None
    try:  # framework Pythons missing certs: fall back to certifi if available
        urllib_request.urlopen("https://ieee-sar-hackathon.ts.r.appspot.com",
                               timeout=10)
    except Exception:
        try:
            import certifi
            ctx = ssl.create_default_context(cafile=certifi.where())
        except ImportError:
            pass
    with urllib_request.urlopen(req, timeout=90, context=ctx) as r:
        print(r.read().decode("utf-8", errors="replace"))


if __name__ == "__main__":
    main()
