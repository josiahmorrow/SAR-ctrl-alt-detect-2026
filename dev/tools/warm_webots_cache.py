"""Pre-fetch every remote asset a world needs into Webots' asset cache.

Webots R2025a pulls PROTOs and textures from raw.githubusercontent.com when a
world loads and caches each under sha1(url). On this machine those downloads
stall part-way (observed 25/11: medium and large hung at load with 17 and 1
assets missing, no controller ever started), and a hung load costs a full run
timeout. Fetching the closure up front makes batch runs independent of that.

    python dev/tools/warm_webots_cache.py [small medium large]

Cache dir: %LOCALAPPDATA%/Cyberbotics/Webots/cache/assets (override WEBOTS_ASSET_CACHE).
Pure fetching: it changes nothing in the worlds or in Webots' settings.
"""
import hashlib
import os
import re
import sys
import urllib.parse
import urllib.request


def _find_env_root(start):
    p = os.path.dirname(os.path.abspath(start))
    while True:
        if (os.path.isdir(os.path.join(p, "worlds")) and os.path.isdir(
                os.path.join(p, "controllers", "sar_marking_supervisor"))):
            return p
        q = os.path.dirname(p)
        if q == p:
            raise RuntimeError("environment root not found")
        p = q


REPO = _find_env_root(__file__)
CACHE = os.environ.get("WEBOTS_ASSET_CACHE") or os.path.join(
    os.environ.get("LOCALAPPDATA", os.path.expanduser("~")),
    "Cyberbotics", "Webots", "cache", "assets")
QUOTED = re.compile(r'"([^"\s]+\.(?:proto|jpg|jpeg|png|hdr|obj|stl|dae|wav|mp3))"', re.I)
WEBOTS_SCHEME = "webots://"
BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/"


def key(url):
    return hashlib.sha1(url.encode("utf-8")).hexdigest()


def fetch(url):
    path = os.path.join(CACHE, key(url))
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return open(path, "rb").read(), False
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    tmp = path + ".part"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)
    return data, True


def refs(url, text):
    out = []
    for m in QUOTED.finditer(text):
        ref = m.group(1)
        if ref.startswith(WEBOTS_SCHEME):
            ref = BASE + ref[len(WEBOTS_SCHEME):]
        elif not ref.startswith(("http://", "https://")):
            if not url.startswith("http"):
                continue                      # local file: Webots resolves it
            ref = urllib.parse.urljoin(url, ref)
        out.append(ref)
    return out


def warm(world):
    wbt = os.path.join(REPO, "worlds", f"{world}_world.wbt")
    todo = refs("", open(wbt, encoding="utf-8").read())
    seen, got, failed = set(), 0, []
    while todo:
        u = todo.pop()
        if u in seen or not u.startswith("http"):
            continue
        seen.add(u)
        try:
            data, new = fetch(u)
        except Exception as e:                    # noqa: BLE001
            failed.append(f"{u}  ({e})")
            continue
        got += new
        if u.lower().endswith(".proto"):
            todo.extend(refs(u, data.decode("utf-8", "replace")))
    print(f"[warm] {world}: {len(seen)} assets, {got} fetched now, "
          f"{len(failed)} failed")
    for f in failed:
        print("   FAILED", f)
    return not failed


if __name__ == "__main__":
    os.makedirs(CACHE, exist_ok=True)
    ok = all([warm(w) for w in (sys.argv[1:] or ["small", "medium", "large"])])
    sys.exit(0 if ok else 1)
