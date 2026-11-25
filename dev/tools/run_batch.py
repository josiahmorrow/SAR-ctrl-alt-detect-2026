"""
Unattended Webots batch runner + official-score parser.

Runs N missions per world with no GUI interaction, captures the supervisor
output and the marking server's report, and appends one row per run to a CSV
so changes can be judged on MEANS rather than single noisy runs (the medium
world varies by 1-2 victims run to run, which is enough to make any single
run useless as evidence).

    .venv/bin/python tools/run_batch.py --world medium --runs 5 --tag baseline
    .venv/bin/python tools/run_batch.py --world all --runs 3 --tag frontier-off

Results: docs/eval/runs/<tag>.csv   Logs: docs/eval/runs/<tag>/<world>_<n>.log

Notes
-----
* Runs are strictly sequential: every mission writes the same sim_logs/ files.
* The supervisor pauses the simulation when the mission ends but never quits
  Webots, and then calls simulationReset() — which would start a SECOND
  mission. We therefore kill Webots as soon as the report (or the summary)
  appears.
* The first run for a world may spend ~2 min on flyover analysis; later runs
  reuse the cached outputs, so batches get much faster after the first.
"""
import argparse
import csv
import os
try:
    import pty
except ImportError:        # Windows
    pty = None
import re
import shutil
import signal
import subprocess
import sys
import time


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

REPO = _find_env_root(__file__)
IS_WIN = os.name == "nt"


def _find_webots():
    if os.environ.get("SAR_WEBOTS"):
        return os.environ["SAR_WEBOTS"]
    if IS_WIN:
        for base in (os.path.join(os.environ.get("LOCALAPPDATA", ""),
                                  "Programs", "Webots"),
                     r"C:\Program Files\Webots"):
            p = os.path.join(base, "msys64", "mingw64", "bin", "webots.exe")
            if os.path.exists(p):
                return p
        return "webots.exe"
    return "/Applications/Webots.app/Contents/MacOS/webots"


WEBOTS = _find_webots()
VENV_PY = (os.path.join(REPO, ".venv", "Scripts", "python.exe") if IS_WIN
           else os.path.join(REPO, ".venv", "bin", "python"))
SOL = os.path.join(REPO, "controllers", "proposed_solution")
# Run CSVs are committed (dev/eval/runs/<tag>.csv); bulky per-run logs are
# git-ignored under dev/eval/runs/<tag>/.
OUT_DIR = os.path.join(SOL, "dev", "eval", "runs")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_pristine                                      # noqa: E402
import warm_webots_cache                                  # noqa: E402
WORLDS = {"small": "small_world.wbt",
          "medium": "medium_world.wbt",
          "large": "large_world.wbt"}

FIELDS = ["tag", "world", "run", "mission_s", "victims_found", "victims_total", "overall",
          "finding", "efficiency", "coordination", "extraction",
          "ratio", "time_score", "confidence", "avg_time_s",
          "dist_score", "dist_per_victim", "robots_contributing",
          "map_score", "victim_est_score", "avg_loc_err_m",
          "total_distance_m", "messages", "coverage_pct", "explore_events",
          "abandoned", "tilt_events", "found_times", "reports",
          "false_reports", "wall_clock_s", "status"]


def _f(pattern, text, cast=float, default=""):
    m = re.search(pattern, text)
    if not m:
        return default
    try:
        return cast(m.group(1))
    except ValueError:
        return default


def parse(text):
    """Pull the official report + our own telemetry out of a run log."""
    row = {
        "overall":     _f(r"OVERALL SCORE:\s*([0-9.]+)", text),
        "finding":     _f(r"Victim Finding\s*│\s*([0-9.]+)", text),
        "efficiency":  _f(r"Efficiency\s*│\s*([0-9.]+)", text),
        "coordination": _f(r"Coordination\s*│\s*([0-9.]+)", text),
        "extraction":  _f(r"Video Information Extraction\s*│\s*([0-9.]+)", text),
        "ratio":       _f(r"Victims Found Ratio Score\s*│\s*([0-9.]+)", text),
        "time_score":  _f(r"Time Score\s*│\s*([0-9.]+)", text),
        "confidence":  _f(r"Confidence Score\s*│\s*([0-9.]+)", text),
        "avg_time_s":  _f(r"Average Time to Find Victim\s*│\s*([0-9.]+)s", text),
        "dist_score":  _f(r"Distance Score\s*│\s*([0-9.]+)", text),
        "dist_per_victim": _f(r"Distance per Victim\s*│\s*([0-9.]+)m", text),
        "robots_contributing": _f(r"Robots with Victims Found\s*│\s*(\d+)", text, int),
        "map_score":   _f(r"Map Estimate Accuracy Score\s*│\s*([0-9.]+)", text),
        "victim_est_score": _f(r"Victim Location Estimate Score\s*│\s*([0-9.]+)", text),
        "avg_loc_err_m": _f(r"Average Location Error\s*│\s*([0-9.]+)m", text),
        "total_distance_m": _f(r"Total Distance Travelled\s*│\s*([0-9.]+)m", text),
        "messages":    _f(r"Total Messages Sent\s*│\s*(\d+)", text, int),
        "victims_found": _f(r"Victims Found:\s*(\d+)/", text, int),
        "victims_total": _f(r"Victims Found:\s*\d+/(\d+)", text, int),
    }
    # Our own instrumentation (works even when the marking server is down)
    cov = re.findall(r"coverage (\d+)%", text)
    row["coverage_pct"]    = max(int(c) for c in cov) if cov else ""
    row["explore_events"]  = len(re.findall(r"Exploring frontier", text))
    row["abandoned"]       = len(re.findall(r"timeout — skipping", text))
    row["tilt_events"]     = len(re.findall(r"Tilt \d+°", text))
    return row


def _newest_marked(after_ts):
    """Path of a Marked_SupervisorLog_*.txt written after `after_ts`, or None.

    Completion is detected from the supervisor's own artefacts rather than from
    stdout: Webots BLOCK-BUFFERS its stdout when redirected, so nothing appears
    in the log file until it exits — which it never does on its own.
    """
    log_dir = os.path.join(REPO, "log")
    newest, newest_ts = None, after_ts
    try:
        for f in os.listdir(log_dir):
            if f.startswith("Marked_SupervisorLog") and f.endswith(".txt"):
                p = os.path.join(log_dir, f)
                ts = os.path.getmtime(p)
                if ts > newest_ts:
                    newest, newest_ts = p, ts
    except OSError:
        pass
    return newest


def restore_pristine(world):
    # Hermetic start: sim_logs/ becomes the shipped preflight's snapshot.
    make_pristine.restore(world)


LOAD_TIMEOUT_S = 150   # world must have started its supervisor by now


EXTRA_ENV = {}         # set from the command line (e.g. SAR_DEBUG_VISION)


MISSION_S = 180       # --mission-s; anything but 180 uses the dev copies


def run_one(world, tag, n, timeout_s=None, pristine=True):
    # pristine=False leaves sim_logs alone so a FOREIGN solution's own preflight
    # runs exactly as its author intended. Used to score another branch on the
    # official server without imposing our flyover pipeline on it.
    if pristine:
        restore_pristine(world)
    log_dir = os.path.join(OUT_DIR, tag)
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"{world}_{n}.log")
    tee_dir = os.path.join(log_dir, f"{world}_{n}_tee")
    shutil.rmtree(tee_dir, ignore_errors=True)
    if MISSION_S == 180:
        wbt = os.path.join(REPO, "worlds", WORLDS[world])        # untouched
    else:
        wbt = os.path.join(REPO, "worlds", f"{world}_world_dev.wbt")
        if not os.path.exists(wbt):
            sys.exit("!! dev worlds missing: run dev/tools/make_dev_supervisor.py")
    if timeout_s is None:
        timeout_s = 300 + 3 * MISSION_S
    cmd = [WEBOTS, "--batch", "--mode=fast", "--minimize",
           "--stdout", "--stderr", wbt]
    # Give Webots a pty: redirected to a plain file it BLOCK-buffers stdout, so
    # nothing is readable until it exits (and it never exits by itself). A pty
    # makes it line-buffered, so our telemetry is captured as the run proceeds.
    t0 = time.time()
    status = "ok"
    marked = None
    chunks = []
    if IS_WIN:
        # No pty on Windows: Webots writes straight into the log file. UTF-8
        # mode keeps the report's arrow glyphs parseable (cp1252 mangles them).
        master = None
        live = open(log_path, "w", encoding="utf-8", errors="replace")
        env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8",
                   PYTHONUNBUFFERED="1", SAR_TEE_DIR=tee_dir, **EXTRA_ENV,
                   **({} if MISSION_S == 180 else
                      {"SAR_MISSION_S": str(MISSION_S)}),
                   WEBOTS_HOME=os.path.normpath(os.path.join(
                       os.path.dirname(WEBOTS), "..", "..", "..")))
        proc = subprocess.Popen(
            cmd, stdout=live, stderr=subprocess.STDOUT, cwd=REPO, env=env,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        master, slave = pty.openpty()
        proc = subprocess.Popen(cmd, stdout=slave, stderr=slave, cwd=REPO,
                                start_new_session=True, close_fds=True)
        os.close(slave)
        os.set_blocking(master, False)
        live = open(log_path, "w", buffering=1)   # written as the run proceeds

    def drain():
        if master is None:
            return
        try:
            data = os.read(master, 65536)
            if data:
                s = data.decode("utf-8", "replace")
                chunks.append(s)
                live.write(s)
        except (BlockingIOError, OSError):
            pass

    done = False
    while time.time() - t0 < timeout_s:
        for _ in range(30):            # drain often, poll files each ~3 s
            drain()
            time.sleep(0.1)
        if proc.poll() is not None:
            done = True
            break
        marked = _newest_marked(t0)
        if marked:
            time.sleep(2)              # let the report finish being written
            drain()
            done = True
            break
        # Load watchdog: a world whose asset download stalls never starts its
        # controllers. Detect that early instead of burning the full timeout.
        if (time.time() - t0 > LOAD_TIMEOUT_S and
                not any(f.startswith("sar_marking_supervisor")
                        for f in (os.listdir(tee_dir) if os.path.isdir(tee_dir)
                                  else []))):
            status = "load-hang"
            break
    if not done and status == "ok":
        status = "timeout"
    try:
        live.close()
        if master is not None:
            os.close(master)
    except OSError:
        pass
    # Kill Webots (and its controller children) — the supervisor never quits,
    # and its simulationReset() would otherwise begin a second mission.
    if IS_WIN:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)
        subprocess.run(["taskkill", "/F", "/T", "/IM", "webots-bin.exe"],
                       capture_output=True)
    else:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        subprocess.run(["pkill", "-f", "Webots.app/Contents/MacOS/webots"],
                       capture_output=True)
    time.sleep(2)
    _collect_tee(tee_dir, log_path)
    with open(log_path, encoding="utf-8", errors="replace") as rf:
        txt = rf.read()
    # The official report lives in log/Marked_*.txt; stdout may have been cut
    # off mid-buffer when we killed Webots, so prefer the file on disk.
    marked = marked or _newest_marked(t0 - 5)
    if marked:
        try:
            with open(marked, encoding="utf-8", errors="replace") as mf:
                txt = txt + "\n" + mf.read()
        except OSError:
            pass
    sup = _newest_supervisor_json(t0 - 5)
    if sup:
        shutil.copy2(sup, os.path.join(log_dir, f"{world}_{n}_supervisor.json"))
    if marked:
        shutil.copy2(marked, os.path.join(log_dir, f"{world}_{n}_marked.txt"))
    row = parse(txt)
    row.update(_truth_fields(sup))
    row.update({"tag": tag, "world": world, "run": n, "status": status,
                "mission_s": MISSION_S,
                "wall_clock_s": round(time.time() - t0, 1)})
    return row


def _collect_tee(tee_dir, log_path):
    if not os.path.isdir(tee_dir):
        return
    with open(log_path, "a", encoding="utf-8") as out:
        for f in sorted(os.listdir(tee_dir)):
            with open(os.path.join(tee_dir, f), encoding="utf-8",
                      errors="replace") as fh:
                out.write(f"\n===== {f} =====\n")
                out.write(fh.read())


def _newest_supervisor_json(after_ts):
    log_dir = os.path.join(REPO, "log")
    best, best_ts = None, after_ts
    try:
        for f in os.listdir(log_dir):
            if f.startswith("SupervisorLog") and f.endswith(".json"):
                p = os.path.join(log_dir, f)
                if os.path.getmtime(p) > best_ts:
                    best, best_ts = p, os.path.getmtime(p)
    except OSError:
        pass
    return best


def _truth_fields(sup):
    """Ground-truth outcome from the supervisor log (offline evaluation only):
    victims found, the sim time of each find, and false reports."""
    import json
    if not sup:
        return {}
    try:
        d = json.load(open(sup))
    except (OSError, ValueError):
        return {}
    vs = d.get("victims", {})
    times = sorted(v["found_time"] for v in vs.values()
                   if v.get("found") and v.get("found_time") is not None)
    reports = [m for m in d.get("messages", [])
               if isinstance(m, dict) and _is_report(m)]
    return {"victims_found": sum(1 for v in vs.values() if v.get("found")),
            "victims_total": len(vs),
            "found_times": " ".join(f"{t:.1f}" for t in times),
            "reports": len(reports),
            "false_reports": max(0, len(reports) - len(times))}


def _is_report(m):
    body = m.get("message", m)
    if isinstance(body, str):
        try:
            import json
            body = json.loads(body)
        except ValueError:
            return False
    return isinstance(body, dict) and bool(body.get("victim_found"))


def _assert_exclusive():
    """Refuse to run if another batch or Webots is already going.

    Missions share sim_logs/, so two concurrent runs corrupt each other's
    deliverables and produce meaningless scores (observed 30/07: a run reported
    0.412 while colliding with a queued batch).
    """
    if IS_WIN:
        out = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq webots-bin.exe", "/NH"],
            capture_output=True, text=True).stdout
        if "webots-bin.exe" in out:
            print("!! Webots already running - refusing to start",
                  file=sys.stderr)
            sys.exit(4)
        return
    out = subprocess.run(["pgrep", "-fl", "MacOS/webots"],
                         capture_output=True, text=True).stdout.strip()
    if out:
        print("!! Webots already running — refusing to start:\n" + out,
              file=sys.stderr)
        sys.exit(4)
    mine = str(os.getpid())
    others = [l for l in subprocess.run(["pgrep", "-fl", "run_batch.py"],
                                        capture_output=True, text=True)
              .stdout.splitlines() if not l.startswith(mine)]
    if others:
        print("!! another batch is running — refusing to start:\n"
              + "\n".join(others), file=sys.stderr)
        sys.exit(4)


def main():
    _assert_exclusive()
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="medium",
                    choices=list(WORLDS) + ["all"])
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--no-pristine", action="store_true",
                    help="don't restore our cached flyover outputs — lets a "
                         "foreign branch's own preflight run untouched")
    ap.add_argument("--debug-vision", action="store_true",
                    help="set SAR_DEBUG_VISION=1: per-robot pose + vision-gate "
                         "telemetry every 0.5 s (diagnosis runs only)")
    ap.add_argument("--mission-s", type=int, default=180,
                    help="mission length; 180 = untouched supervisor and worlds, "
                         "anything else = time-curriculum dev copies")
    args = ap.parse_args()
    global MISSION_S
    MISSION_S = args.mission_s
    if args.debug_vision:
        EXTRA_ENV["SAR_DEBUG_VISION"] = "1"

    worlds = list(WORLDS) if args.world == "all" else [args.world]
    for world in worlds:                 # see warm_webots_cache.py
        warm_webots_cache.warm(world)
    os.makedirs(OUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUT_DIR, f"{args.tag}.csv")
    new = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as cf:
        w = csv.DictWriter(cf, fieldnames=FIELDS)
        if new:
            w.writeheader()
        for world in worlds:
            for n in range(1, args.runs + 1):
                for attempt in range(3):
                    row = run_one(world, args.tag, n,
                                  pristine=not args.no_pristine)
                    if row["status"] != "load-hang":
                        break
                    print(f"[{args.tag}] {world} run {n}: world load hung - "
                          f"re-warming asset cache and retrying", flush=True)
                    warm_webots_cache.warm(world)
                w.writerow(row)
                cf.flush()
                print(f"[{args.tag}] {world} run {n}: "
                      f"victims {row['victims_found']}/{row['victims_total']} "
                      f"overall {row['overall']} "
                      f"cov {row['coverage_pct']}% "
                      f"({row['wall_clock_s']}s wall, {row['status']})",
                      flush=True)
    print(f"\nWrote {csv_path}")


if __name__ == "__main__":
    main()
