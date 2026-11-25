#!/usr/bin/env python3
"""Offline evaluation harness for the Video Information Extraction score.

Replicates the ground truth that ``sar_marking_supervisor.py`` builds at runtime,
straight from a ``.wbt`` world file — no Webots needed:

  * ground-truth wall map  (same algorithm as ``_generate_ground_truth_map``:
    Wall/Window/Door nodes, skip z > 1.0, filled black polygons, 600x600 @ 0.05 m/px,
    centred on the bounding box of all wall corners, +X right / +Y up)
  * ground-truth victim positions (the DEF VICTIM_MARKER solid inside Victim.proto,
    i.e. body origin + per-model marker offset rotated by the victim rotation)

and scores our generated ``sim_logs/map_estimate.png`` / ``victim_location_estimates.csv``
against them (IoU / F1 / pixel accuracy, best-shift diagnostic, victim distances and a
count-penalised proxy score).

Usage:
  python tools/eval_harness.py gt    worlds/small_world.wbt [--out-dir docs/eval]
  python tools/eval_harness.py score worlds/small_world.wbt \
      [--map controllers/proposed_solution/sim_logs/map_estimate.png] \
      [--csv controllers/proposed_solution/sim_logs/victim_location_estimates.csv] \
      [--out-dir docs/eval]

Dev tool only — NOT part of the competition submission runtime.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import re
import sys

import numpy as np
from PIL import Image, ImageDraw

# --------------------------------------------------------------------------- #
# .wbt parsing (minimal VRML subset)
# --------------------------------------------------------------------------- #

_TOKEN_RE = re.compile(r'"(?:[^"\\]|\\.)*"|[{}\[\]]|[^\s{}\[\],]+')


class UseRef:
    def __init__(self, name: str):
        self.name = name


class Node:
    __slots__ = ("type", "def_name", "fields")

    def __init__(self, type_: str, def_name: str | None = None):
        self.type = type_
        self.def_name = def_name
        self.fields: list[tuple[str, object]] = []

    def get(self, name: str, default=None):
        for n, v in self.fields:
            if n == name:
                return v
        return default

    def child_nodes(self) -> list["Node"]:
        v = self.get("children") or []
        if isinstance(v, Node):
            return [v]
        if isinstance(v, list):
            return [c for c in v if isinstance(c, Node)]
        return []

    def __repr__(self):
        return f"<{self.type} {self.def_name or ''} {self.get('name') or ''}>"


def _strip_comments(text: str) -> str:
    out_lines = []
    for line in text.splitlines():
        out, in_str, i = [], False, 0
        while i < len(line):
            c = line[i]
            if c == '"' and (i == 0 or line[i - 1] != "\\"):
                in_str = not in_str
            if c == "#" and not in_str:
                break
            out.append(c)
            i += 1
        out_lines.append("".join(out))
    return "\n".join(out_lines)


def _is_number(tok: str) -> bool:
    try:
        float(tok)
        return True
    except ValueError:
        return False


class Parser:
    def __init__(self, text: str):
        self.toks = _TOKEN_RE.findall(_strip_comments(text))
        self.i = 0

    def peek(self) -> str | None:
        return self.toks[self.i] if self.i < len(self.toks) else None

    def next(self) -> str:
        t = self.toks[self.i]
        self.i += 1
        return t

    def parse_world(self) -> list[Node]:
        nodes = []
        while self.peek() is not None:
            if self.peek() == "EXTERNPROTO":  # `EXTERNPROTO "url"` header lines
                self.next()
                self.next()
                continue
            nodes.append(self.parse_node())
        return nodes

    def parse_node(self) -> Node:
        tok = self.next()
        def_name = None
        if tok == "DEF":
            def_name = self.next()
            tok = self.next()
        node = Node(tok, def_name)
        if self.peek() != "{":
            raise ValueError(f"expected '{{' after node type {tok!r} (tok {self.i})")
        self.next()  # {
        while self.peek() != "}":
            fname = self.next()
            if fname == "hidden":  # e.g. `hidden position_6_0 0.01`
                fname = "hidden:" + self.next()
            node.fields.append((fname, self.parse_value()))
        self.next()  # }
        return node

    def parse_value(self):
        t = self.peek()
        if t == "[":
            self.next()
            items = []
            while self.peek() != "]":
                items.append(self.parse_value())
            self.next()
            # flatten number runs: [0.21 0.21] parses as one [0.21, 0.21] entry
            if len(items) == 1 and isinstance(items[0], list) and all(
                isinstance(x, float) for x in items[0]
            ):
                return items[0]
            return items
        if t.startswith('"'):
            self.next()
            return t[1:-1]
        if t in ("TRUE", "FALSE"):
            self.next()
            return t == "TRUE"
        if t == "NULL":
            self.next()
            return None
        if t == "USE":
            self.next()
            return UseRef(self.next())
        if _is_number(t):
            nums = []
            while self.peek() is not None and _is_number(self.peek()):
                nums.append(float(self.next()))
            return nums
        return self.parse_node()


# --------------------------------------------------------------------------- #
# scene-tree helpers (replicates supervisor traversal semantics)
# --------------------------------------------------------------------------- #

# Proto default field values that matter when a world file omits the field.
# (size defaults verified against the Webots R2025a apartment_structure protos)
PROTO_DEFAULTS = {
    "Victim": {"rotation": [0.0, 0.0, 1.0, 1.5708]},
    "Wall": {"size": [0.2, 1.0, 2.4]},
    "Window": {"size": [0.2, 0.8, 2.4]},
    "Door": {"size": [0.2, 1.0, 2.4]},
}

# DEF VICTIM_MARKER local offsets per model (verbatim from protos/Victim.proto)
VICTIM_MARKER_OFFSETS = {
    "man_1": (0.0, -0.4, 0.0),
    "man_2": (0.0, -1.3, 0.0),
    "man_3": (0.0, 1.3, 0.0),
    "boy_1": (0.0, -0.2, 0.0),
    "boy_2": (0.0, 0.7, 0.0),
    "boy_3": (0.0, -0.2, 0.0),
    "woman_1": (0.0, 0.9, 0.0),
    "woman_2": (0.2, 0.7, 0.0),
    "woman_3": (0.0, -0.3, 0.0),
    "girl_1": (0.0, 0.7, 0.0),
    "girl_2": (-0.1, 0.4, 0.0),
    "girl_3": (0.0, -0.2, 0.0),
}
VICTIM_MARKER_DEFAULT = (-1.0, 0.0, 0.0)


def rot_matrix(axis_angle) -> np.ndarray:
    x, y, z, a = axis_angle
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0.0:
        return np.eye(3)
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ]
    )


def node_local_pose(node: Node) -> tuple[np.ndarray, np.ndarray]:
    defaults = PROTO_DEFAULTS.get(node.type, {})
    t = node.get("translation", defaults.get("translation", [0.0, 0.0, 0.0]))
    r = node.get("rotation", defaults.get("rotation"))
    R = rot_matrix(r) if r else np.eye(3)
    return np.asarray(t, dtype=float), R


def walk(nodes: list[Node]):
    """Yield (node, world_t, world_R) for every node, depth-first through `children`
    fields — mirroring the supervisor's _find_nodes_by_name recursion."""
    stack = [(n, np.zeros(3), np.eye(3)) for n in reversed(nodes)]
    while stack:
        node, pt, pR = stack.pop()
        lt, lR = node_local_pose(node)
        wt = pt + pR @ lt
        wR = pR @ lR
        yield node, wt, wR
        for c in reversed(node.child_nodes()):
            stack.append((c, wt, wR))


def load_world(wbt_path: str):
    with open(wbt_path, "r", encoding="utf-8") as f:
        nodes = Parser(f.read()).parse_world()

    walls, victims, origin = [], [], None
    for node, wt, wR in walk(nodes):
        if node.type in ("Wall", "Window", "Door"):
            size = node.get("size") or PROTO_DEFAULTS[node.type]["size"]
            walls.append(
                {
                    "type": node.type,
                    "name": node.get("name"),
                    "pos": wt,
                    "R": wR,
                    "size": list(size),
                }
            )
        elif node.type == "Victim":
            model = node.get("model", "man")
            off = np.asarray(
                VICTIM_MARKER_OFFSETS.get(model.lower(), VICTIM_MARKER_DEFAULT)
            )
            marker = wt + wR @ off
            victims.append(
                {
                    "name": node.get("name", "victim"),
                    "model": model,
                    "body_world": wt.copy(),
                    "marker_world": marker,
                }
            )
        elif node.type == "OriginMarker":
            origin = wt.copy()

    return {"walls": walls, "victims": victims, "origin": origin}


# --------------------------------------------------------------------------- #
# ground-truth map (byte-for-byte replica of supervisor logic)
# --------------------------------------------------------------------------- #

MAP_W, MAP_H, RESOLUTION = 600, 600, 0.05


def wall_corners(wall) -> list[tuple[float, float]]:
    dx, dy = wall["size"][0] / 2.0, wall["size"][1] / 2.0
    pos, R = wall["pos"], wall["R"]
    corners = []
    for lx, ly in ((-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)):
        v = R @ np.array([lx, ly, 0.0])
        corners.append((pos[0] + v[0], pos[1] + v[1]))
    return corners


def generate_ground_truth_map(walls):
    """Replicates SARMarkingSupervisor._generate_ground_truth_map."""
    wall_polygons, all_points, skipped = [], [], 0
    for wall in walls:
        if wall["pos"][2] > 1.0:
            skipped += 1
            continue
        corners = wall_corners(wall)
        wall_polygons.append(corners)
        all_points.extend(corners)

    if not wall_polygons:
        raise ValueError("no walls found")

    pts = np.array(all_points)
    all_min_x, all_max_x = pts[:, 0].min(), pts[:, 0].max()
    all_min_y, all_max_y = pts[:, 1].min(), pts[:, 1].max()
    center_x = (all_min_x + all_max_x) / 2
    center_y = (all_min_y + all_max_y) / 2

    world_width = MAP_W * RESOLUTION
    world_height = MAP_H * RESOLUTION
    world_min_x = center_x - world_width / 2
    world_max_y = center_y + world_height / 2

    image = Image.new("L", (MAP_W, MAP_H), 255)
    draw = ImageDraw.Draw(image)
    for corners in wall_polygons:
        img_corners = []
        for wx, wy in corners:
            img_x = int((wx - world_min_x) / RESOLUTION)
            img_y = int((world_max_y - wy) / RESOLUTION)
            img_corners.append((img_x, img_y))
        if len(img_corners) >= 3:
            draw.polygon(img_corners, fill=0)

    meta = {
        "center": (center_x, center_y),
        "world_min_x": world_min_x,
        "world_max_y": world_max_y,
        "bbox": (all_min_x, all_min_y, all_max_x, all_max_y),
        "skipped_elevated": skipped,
        "n_polygons": len(wall_polygons),
    }
    return np.array(image), meta


def world_to_px(meta, wx, wy) -> tuple[int, int]:
    return (
        int((wx - meta["world_min_x"]) / RESOLUTION),
        int((meta["world_max_y"] - wy) / RESOLUTION),
    )


# --------------------------------------------------------------------------- #
# scoring
# --------------------------------------------------------------------------- #

def binarize(img: Image.Image) -> np.ndarray:
    return np.asarray(img.convert("L")) < 128  # True = wall


def iou_f1_acc(est: np.ndarray, gt: np.ndarray):
    inter = np.logical_and(est, gt).sum()
    union = np.logical_or(est, gt).sum()
    iou = inter / union if union else 0.0
    denom = est.sum() + gt.sum()
    f1 = 2 * inter / denom if denom else 0.0
    acc = (est == gt).mean()
    return iou, f1, acc


def shifted_iou(est: np.ndarray, gt: np.ndarray, dx: int, dy: int) -> float:
    """IoU after translating the estimate by (dx, dy) pixels."""
    H, W = gt.shape
    xs1, xs2 = max(0, dx), min(W, W + dx)
    ys1, ys2 = max(0, dy), min(H, H + dy)
    if xs1 >= xs2 or ys1 >= ys2:
        return 0.0
    sub_gt = gt[ys1:ys2, xs1:xs2]
    sub_est = est[ys1 - dy : ys2 - dy, xs1 - dx : xs2 - dx]
    inter = np.logical_and(sub_gt, sub_est).sum()
    union = est.sum() + gt.sum() - inter
    return inter / union if union else 0.0


def best_shift(est: np.ndarray, gt: np.ndarray, search: int = 120):
    best = (0.0, 0, 0)
    for dy in range(-search, search + 1, 4):
        for dx in range(-search, search + 1, 4):
            v = shifted_iou(est, gt, dx, dy)
            if v > best[0]:
                best = (v, dx, dy)
    _, bx, by = best
    for dy in range(by - 3, by + 4):
        for dx in range(bx - 3, bx + 4):
            v = shifted_iou(est, gt, dx, dy)
            if v > best[0]:
                best = (v, dx, dy)
    return best  # (iou, dx, dy)


def check_map_format(img: Image.Image) -> list[str]:
    problems = []
    if img.size != (MAP_W, MAP_H):
        problems.append(f"size is {img.size}, expected {(MAP_W, MAP_H)}")
    arr = np.asarray(img.convert("L"))
    vals = np.unique(arr)
    non_pure = [int(v) for v in vals if v not in (0, 255)]
    if non_pure:
        problems.append(
            f"{len(non_pure)} grey levels present (e.g. {non_pure[:6]}) — must be pure 0/255"
        )
    return problems


def match_victims(est_xy: np.ndarray, gt_xy: np.ndarray):
    """Optimal assignment for small n (exhaustive), returns list of (ei, gi, dist)."""
    n_e, n_g = len(est_xy), len(gt_xy)
    if n_e == 0 or n_g == 0:
        return []
    D = np.linalg.norm(est_xy[:, None, :] - gt_xy[None, :, :], axis=2)
    k = min(n_e, n_g)
    best_pairs, best_cost = None, None
    idx_small, idx_large, transposed = (
        (range(n_e), range(n_g), False) if n_e <= n_g else (range(n_g), range(n_e), True)
    )
    for perm in itertools.permutations(idx_large, k):
        pairs = list(zip(idx_small, perm))
        cost = sum(D[b, a] if transposed else D[a, b] for a, b in pairs)
        if best_cost is None or cost < best_cost:
            best_cost, best_pairs = cost, pairs
    out = []
    for a, b in best_pairs:
        ei, gi = (b, a) if transposed else (a, b)
        out.append((ei, gi, float(D[ei, gi])))
    return out


def victim_report(est_xy: np.ndarray, gt: list[dict], origin: np.ndarray):
    gt_xy = np.array([[v["marker_world"][0] - origin[0], v["marker_world"][1] - origin[1]] for v in gt])
    pairs = match_victims(est_xy, gt_xy)
    lines = []
    lines.append(f"  estimates: {len(est_xy)}   ground truth victims: {len(gt_xy)}")
    per_pair_scores = []
    for ei, gi, d in sorted(pairs, key=lambda p: p[1]):
        v = gt[gi]
        lines.append(
            f"  est ({est_xy[ei][0]:+7.2f},{est_xy[ei][1]:+7.2f})  ->  "
            f"{v['name']:<8} marker ({gt_xy[gi][0]:+7.2f},{gt_xy[gi][1]:+7.2f})   dist {d:5.2f} m"
        )
        per_pair_scores.append(max(0.0, 1.0 - d / 1.0))
    matched_g = {gi for _, gi, _ in pairs}
    for gi, v in enumerate(gt):
        if gi not in matched_g:
            lines.append(f"  MISSED   {v['name']:<8} marker ({gt_xy[gi][0]:+7.2f},{gt_xy[gi][1]:+7.2f})")
    matched_e = {ei for ei, _, _ in pairs}
    for ei in range(len(est_xy)):
        if ei not in matched_e:
            lines.append(f"  SPURIOUS est ({est_xy[ei][0]:+7.2f},{est_xy[ei][1]:+7.2f})")
    denom = max(len(est_xy), len(gt_xy), 1)
    proxy = sum(per_pair_scores) / denom
    dists = [d for _, _, d in pairs]
    lines.append(
        f"  mean matched dist: {np.mean(dists):.2f} m   "
        f"proxy score (1-d clipped, /max-count): {proxy:.3f}"
        if dists
        else "  no matches"
    )
    return "\n".join(lines), proxy


# --------------------------------------------------------------------------- #
# composite visualization
# --------------------------------------------------------------------------- #

def composite(gt_arr: np.ndarray, est_img: Image.Image | None, meta, world, est_xy):
    """GT vs estimate overlay: GT-only walls black, est-only red, overlap green.
    GT victim markers blue dots, victim estimates orange crosses."""
    gt = gt_arr < 128
    H, W = gt.shape
    rgb = np.full((H, W, 3), 255, np.uint8)
    if est_img is not None:
        est_l = np.asarray(est_img.convert("L").resize((W, H), Image.NEAREST))
        est = est_l < 128
        rgb[gt & ~est] = (0, 0, 0)
        rgb[est & ~gt] = (220, 40, 40)
        rgb[est & gt] = (40, 170, 40)
    else:
        rgb[gt] = (0, 0, 0)
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    origin = world["origin"]
    for v in world["victims"]:
        px, py = world_to_px(meta, v["marker_world"][0], v["marker_world"][1])
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], outline=(30, 60, 255), width=3)
    ox, oy = world_to_px(meta, origin[0], origin[1])
    draw.rectangle([ox - 4, oy - 4, ox + 4, oy + 4], outline=(150, 0, 200), width=2)
    for ex, ey in est_xy:
        px, py = world_to_px(meta, ex + origin[0], ey + origin[1])
        draw.line([px - 6, py - 6, px + 6, py + 6], fill=(255, 140, 0), width=3)
        draw.line([px - 6, py + 6, px + 6, py - 6], fill=(255, 140, 0), width=3)
    return img


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def world_name(wbt_path: str) -> str:
    return os.path.splitext(os.path.basename(wbt_path))[0]


def cmd_gt(args):
    world = load_world(args.world)
    name = world_name(args.world)
    os.makedirs(args.out_dir, exist_ok=True)

    n_by_type = {}
    for w in world["walls"]:
        n_by_type[w["type"]] = n_by_type.get(w["type"], 0) + 1
    print(f"[{name}] wall-like nodes: {n_by_type}  (total {len(world['walls'])})")

    gt_arr, meta = generate_ground_truth_map(world["walls"])
    print(
        f"[{name}] GT map: {meta['n_polygons']} polygons drawn, "
        f"{meta['skipped_elevated']} skipped (z>1), wall bbox {tuple(round(v,2) for v in meta['bbox'])}, "
        f"centre {tuple(round(v,2) for v in meta['center'])}"
    )
    out_png = os.path.join(args.out_dir, f"gt_map_{name}.png")
    Image.fromarray(gt_arr).save(out_png)
    print(f"[{name}] wrote {out_png}")

    origin = world["origin"]
    print(f"[{name}] OriginMarker world position: {tuple(round(v,3) for v in origin)}")
    out_csv = os.path.join(args.out_dir, f"gt_victims_{name}.csv")
    with open(out_csv, "w", encoding="utf-8") as f:
        f.write("name,model,body_world_x,body_world_y,marker_world_x,marker_world_y,marker_aruco_x,marker_aruco_y\n")
        for v in world["victims"]:
            ax, ay = v["marker_world"][0] - origin[0], v["marker_world"][1] - origin[1]
            f.write(
                f"{v['name']},{v['model']},{v['body_world'][0]:.4f},{v['body_world'][1]:.4f},"
                f"{v['marker_world'][0]:.4f},{v['marker_world'][1]:.4f},{ax:.4f},{ay:.4f}\n"
            )
            print(
                f"[{name}]   {v['name']:<8} {v['model']:<8} body ({v['body_world'][0]:+7.2f},{v['body_world'][1]:+7.2f})"
                f"  marker ({v['marker_world'][0]:+7.2f},{v['marker_world'][1]:+7.2f})"
                f"  aruco ({ax:+6.2f},{ay:+6.2f})"
            )
    print(f"[{name}] wrote {out_csv}")


def cmd_score(args):
    world = load_world(args.world)
    name = world_name(args.world)
    os.makedirs(args.out_dir, exist_ok=True)
    gt_arr, meta = generate_ground_truth_map(world["walls"])

    print(f"=== {name} ===")

    est_img = None
    if os.path.exists(args.map):
        est_img = Image.open(args.map)
        problems = check_map_format(est_img)
        for p in problems:
            print(f"  [format] {p}")
        est = binarize(est_img)
        if est.shape != gt_arr.shape:
            est_img_r = est_img.convert("L").resize((MAP_W, MAP_H), Image.NEAREST)
            est = binarize(est_img_r)
        gt = gt_arr < 128
        iou, f1, acc = iou_f1_acc(est, gt)
        print(f"  MAP  IoU {iou:.4f}   F1 {f1:.4f}   pixel-acc {acc:.4f}")
        print(f"       wall px: est {int(est.sum())}  gt {int(gt.sum())}")
        b_iou, bdx, bdy = best_shift(est, gt, search=args.search)
        print(
            f"       best-shift IoU {b_iou:.4f} at shift ({bdx:+d},{bdy:+d}) px "
            f"= ({bdx*RESOLUTION:+.2f},{-bdy*RESOLUTION:+.2f}) m  "
            f"(gain vs unshifted: {b_iou - iou:+.4f})"
        )
    else:
        print(f"  MAP  estimate not found: {args.map}")

    est_xy = np.zeros((0, 2))
    if os.path.exists(args.csv) and os.path.getsize(args.csv) > 0:
        rows = []
        with open(args.csv, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    parts = line.split(",")
                    rows.append((float(parts[0]), float(parts[1])))
        est_xy = np.array(rows) if rows else np.zeros((0, 2))
        rep, proxy = victim_report(est_xy, world["victims"], world["origin"])
        print("  VICTIMS")
        print(rep)
    else:
        print(f"  VICTIMS  estimate CSV missing/empty: {args.csv}")
        rep, proxy = victim_report(est_xy, world["victims"], world["origin"])
        print(rep)

    comp = composite(gt_arr, est_img, meta, world, est_xy)
    out = os.path.join(args.out_dir, f"compare_{name}.png")
    comp.save(out)
    print(f"  wrote {out}  (black=GT-only wall, red=est-only, green=overlap, blue=GT victim marker, orange X=victim est, purple=origin)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    ap_gt = sub.add_parser("gt", help="generate ground truth map + victim positions")
    ap_gt.add_argument("world")
    ap_gt.add_argument("--out-dir", default="docs/eval")
    ap_gt.set_defaults(func=cmd_gt)

    ap_sc = sub.add_parser("score", help="score estimate map + victim csv against GT")
    ap_sc.add_argument("world")
    ap_sc.add_argument("--map", default="controllers/proposed_solution/sim_logs/map_estimate.png")
    ap_sc.add_argument("--csv", default="controllers/proposed_solution/sim_logs/victim_location_estimates.csv")
    ap_sc.add_argument("--out-dir", default="docs/eval")
    ap_sc.add_argument("--search", type=int, default=120)
    ap_sc.set_defaults(func=cmd_score)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
