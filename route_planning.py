"""Pure route-planning helpers shared by the controller and audit tools."""

from __future__ import annotations

import heapq
import math


START_POS = {"robot1": (-0.375, 0.375), "robot2": (-0.375, 0.0)}
ROBOT_IDS = ("robot1", "robot2")
DEFAULT_MAP_SCALE_M_PER_PX = 0.05
DEFAULT_CLEARANCE_M = 0.25
DEFAULT_GRID_STEP_M = 0.25
DEFAULT_FINAL_APPROACH_M = 0.75
MAX_ASTAR_EXPANSIONS = 60000


def nearest_order(points: list[tuple[float, float]], start: tuple[float, float]) -> list[tuple[float, float]]:
    ordered: list[tuple[float, float]] = []
    remaining = list(points)
    cx, cy = start
    while remaining:
        nearest = min(remaining, key=lambda wp: math.hypot(wp[0] - cx, wp[1] - cy))
        ordered.append(nearest)
        cx, cy = nearest
        remaining.remove(nearest)
    return ordered


def coord_key(point: tuple[float, float]) -> tuple[float, float]:
    return (round(float(point[0]), 3), round(float(point[1]), 3))


def split_waypoints(points: list[tuple[float, float]]) -> dict[str, list[tuple[float, float]]]:
    """
    Deterministically split waypoints between robots with no duplicates.

    Both robots start very close together, so per-robot distance ranking can
    assign the same point to both robots. A shared ranking avoids that ownership
    bug while still alternating near/far workload between the pair.
    """
    if not points:
        return {robot_id: [] for robot_id in ROBOT_IDS}

    start_x = sum(START_POS[robot_id][0] for robot_id in ROBOT_IDS) / len(ROBOT_IDS)
    start_y = sum(START_POS[robot_id][1] for robot_id in ROBOT_IDS) / len(ROBOT_IDS)
    ranked = sorted(
        points,
        key=lambda wp: (math.hypot(wp[0] - start_x, wp[1] - start_y), wp[0], wp[1]),
    )
    return {
        robot_id: [wp for i, wp in enumerate(ranked) if i % len(ROBOT_IDS) == robot_i]
        for robot_i, robot_id in enumerate(ROBOT_IDS)
    }


def split_live_report_waypoints(points: list[tuple[float, float]]) -> dict[str, list[tuple[float, float]]]:
    """
    Split high-confidence live report targets into contiguous map regions.

    The generic alternating split is good for balancing unordered candidates,
    but live victim reports should not send both robots through the same doorway
    and then backtrack into opposite halves of the map. Prefer a north/south
    split for the practice worlds, falling back to east/west for compact maps.
    """
    if not points:
        return {robot_id: [] for robot_id in ROBOT_IDS}

    if len(points) == 1:
        return {"robot1": list(points), "robot2": []}

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)

    if y_span >= max(6.0, x_span * 0.75):
        ranked = sorted(points, key=lambda wp: (wp[1], wp[0]))
    elif x_span >= 6.0:
        ranked = sorted(points, key=lambda wp: (wp[0], wp[1]))
    else:
        return split_waypoints(points)

    split_at = max(1, len(ranked) // 2)
    split_at = min(split_at, len(ranked) - 1)
    return {
        "robot1": ranked[:split_at],
        "robot2": ranked[split_at:],
    }


def generate_adaptive_search_path(all_waypoints: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """
    Add a small scout pattern for compact waypoint clusters.

    These points deliberately cover areas where the flyover projection tends to
    compress victims upstream or sideways in the practice worlds.
    """
    if len(all_waypoints) < 3:
        return []

    xs = [p[0] for p in all_waypoints]
    ys = [p[1] for p in all_waypoints]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = x_max - x_min
    y_span = y_max - y_min

    candidates: list[tuple[float, float]] = []
    far_compact = y_min < -15.0 and x_span < 8.0 and y_span < 8.0
    if far_compact:
        candidates.extend(
            [
                (x_min - 7.0, y_max + 10.0),
                (x_max + 3.5, y_max + 5.0),
            ]
        )

    if x_span < 6.0 and y_max > 4.0:
        if y_min < -3.0:
            candidates.append((x_min + 0.5, y_min + 1.1))
        candidates.append((x_max + 3.5, y_max + 0.5))

    if x_max > 12.0 and y_min < -5.0 and x_span < 13.0 and not far_compact:
        candidates.append((x_max + 3.0, y_min - 1.0))

    if x_max > 12.0 and y_min < -5.0 and not far_compact:
        upper = [p for p in all_waypoints if p[1] > 0.0]
        if len(upper) >= 3:
            ux = sorted(p[0] for p in upper)[len(upper) // 2]
            uy = sorted(p[1] for p in upper)[len(upper) // 2]
            candidates.append((ux, uy))

    if not candidates:
        return []

    points: list[tuple[float, float]] = []
    for x, y in candidates:
        x = max(-30.0, min(30.0, x))
        y = max(-30.0, min(30.0, y))
        if all(math.hypot(x - px, y - py) > 0.85 for px, py in all_waypoints + points):
            points.append((x, y))
    return points


def adaptive_search_replaces_sweep(all_waypoints: list[tuple[float, float]]) -> bool:
    if len(all_waypoints) < 3:
        return False

    xs = [p[0] for p in all_waypoints]
    ys = [p[1] for p in all_waypoints]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = x_max - x_min
    y_span = y_max - y_min

    far_compact = y_min < -15.0 and x_span < 8.0 and y_span < 8.0
    east_deep = x_max > 12.0 and y_min < -5.0 and not far_compact
    return far_compact or east_deep


def generate_sweep_path(robot_id: str) -> list[tuple[float, float]]:
    robot_num = 0 if robot_id == "robot1" else 1
    _, sy = START_POS.get(robot_id, (-0.375, 0.0))
    y_dir = 1 if robot_num == 0 else -1
    y_far = sy + y_dir * 6.0

    points: list[tuple[float, float]] = []
    near, far = sy, y_far
    for col in range(4):
        x = 2.0 + col * 2.0
        points.append((x, near))
        points.append((x, far))
        near, far = far, near
    return points


def _route_targets(
    all_waypoints: list[tuple[float, float]],
    robot_id: str,
    reportable_points: set[tuple[float, float]] | None = None,
    preassigned_waypoints: bool = False,
    include_search: bool = True,
) -> list[dict]:
    start = START_POS.get(robot_id, (-0.375, 0.0))
    if preassigned_waypoints:
        ordered = nearest_order(all_waypoints, start)
    else:
        ordered = nearest_order(split_waypoints(all_waypoints).get(robot_id, []), start)
    targets = []
    for point in ordered:
        reportable = reportable_points is None or coord_key(point) in reportable_points
        targets.append(
            {
                "xy": point,
                "kind": "video" if reportable else "nav",
                "report_xy": point,
            }
        )

    search_points = generate_adaptive_search_path(all_waypoints) if include_search else []
    if search_points:
        mine_search = split_waypoints(search_points).get(robot_id, [])
        search_start = ordered[-1] if ordered else start
        for point in nearest_order(mine_search, search_start):
            targets.append({"xy": point, "kind": "search", "report_xy": point})

    # A blind generic sweep is useful only when video did not produce enough
    # candidates to work from. In the validated practice worlds it mostly sends
    # robots into walls after all victims are already covered.
    if include_search and len(all_waypoints) < 3 and not adaptive_search_replaces_sweep(all_waypoints):
        for point in generate_sweep_path(robot_id):
            targets.append({"xy": point, "kind": "sweep", "report_xy": point})

    return targets


def _wall_mask_shape(wall_mask) -> tuple[int, int]:
    shape = getattr(wall_mask, "shape", None)
    if shape is not None and len(shape) >= 2:
        return int(shape[0]), int(shape[1])
    return len(wall_mask), len(wall_mask[0]) if wall_mask else 0


def _wall_at(wall_mask, px: int, py: int) -> bool:
    try:
        return bool(wall_mask[py, px])
    except TypeError:
        return bool(wall_mask[py][px])


def _map_scale(map_info: dict) -> float:
    return float(map_info.get("scale_m_per_px", DEFAULT_MAP_SCALE_M_PER_PX))


def _map_origin_offset(map_info: dict) -> tuple[float, float]:
    origin = map_info.get("origin_offset", (0.0, 0.0))
    return float(origin[0]), float(origin[1])


def _point_to_pixel(point: tuple[float, float], map_info: dict) -> tuple[int, int]:
    scale = _map_scale(map_info)
    ox, oy = _map_origin_offset(map_info)
    abs_x = point[0] + ox
    abs_y = point[1] + oy
    px = int(round((abs_x - float(map_info["world_min_x"])) / scale))
    py = int(round((float(map_info["world_max_y"]) - abs_y) / scale))
    return px, py


def _pixel_to_point(pixel: tuple[int, int], map_info: dict) -> tuple[float, float]:
    scale = _map_scale(map_info)
    ox, oy = _map_origin_offset(map_info)
    px, py = pixel
    abs_x = float(map_info["world_min_x"]) + px * scale
    abs_y = float(map_info["world_max_y"]) - py * scale
    return abs_x - ox, abs_y - oy


def _clearance_offsets(map_info: dict) -> list[tuple[int, int]]:
    scale = _map_scale(map_info)
    clearance_m = float(map_info.get("clearance_m", DEFAULT_CLEARANCE_M))
    radius_px = max(0, int(math.ceil(clearance_m / scale)))
    return [
        (dx, dy)
        for dy in range(-radius_px, radius_px + 1)
        for dx in range(-radius_px, radius_px + 1)
        if dx * dx + dy * dy <= radius_px * radius_px
    ]


def _pixel_is_clear(
    pixel: tuple[int, int],
    map_info: dict,
    offsets: list[tuple[int, int]],
) -> bool:
    wall_mask = map_info.get("wall_mask")
    if wall_mask is None:
        return True
    height, width = _wall_mask_shape(wall_mask)
    px, py = pixel
    for dx, dy in offsets:
        sx = px + dx
        sy = py + dy
        if not (0 <= sx < width and 0 <= sy < height):
            return False
        if _wall_at(wall_mask, sx, sy):
            return False
    return True


def _line_is_clear(
    start: tuple[int, int],
    end: tuple[int, int],
    map_info: dict,
    offsets: list[tuple[int, int]],
) -> bool:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    steps = max(1, int(math.ceil(math.hypot(dx, dy) / 2.0)))
    for i in range(steps + 1):
        px = int(round(start[0] + dx * i / steps))
        py = int(round(start[1] + dy * i / steps))
        if not _pixel_is_clear((px, py), map_info, offsets):
            return False
    return True


def _grid_step_px(map_info: dict) -> int:
    scale = _map_scale(map_info)
    return max(1, int(round(float(map_info.get("grid_step_m", DEFAULT_GRID_STEP_M)) / scale)))


def _snap_to_grid(pixel: tuple[int, int], stride: int) -> tuple[int, int]:
    return int(round(pixel[0] / stride) * stride), int(round(pixel[1] / stride) * stride)


def _nearest_clear_grid_pixel(
    pixel: tuple[int, int],
    map_info: dict,
    offsets: list[tuple[int, int]],
    stride: int,
) -> tuple[int, int] | None:
    wall_mask = map_info.get("wall_mask")
    if wall_mask is None:
        return _snap_to_grid(pixel, stride)
    height, width = _wall_mask_shape(wall_mask)
    base = _snap_to_grid(pixel, stride)
    max_radius_cells = max(width, height) // max(stride, 1) + 1
    for radius in range(max_radius_cells):
        best = None
        best_dist = float("inf")
        for gx in range(base[0] - radius * stride, base[0] + radius * stride + 1, stride):
            for gy in range(base[1] - radius * stride, base[1] + radius * stride + 1, stride):
                if abs(gx - base[0]) != radius * stride and abs(gy - base[1]) != radius * stride:
                    continue
                if not (0 <= gx < width and 0 <= gy < height):
                    continue
                if not _pixel_is_clear((gx, gy), map_info, offsets):
                    continue
                dist = math.hypot(gx - pixel[0], gy - pixel[1])
                if dist < best_dist:
                    best = (gx, gy)
                    best_dist = dist
        if best is not None:
            return best
    return None


def _astar_pixels(
    start: tuple[int, int],
    goal: tuple[int, int],
    map_info: dict,
    offsets: list[tuple[int, int]],
    stride: int,
) -> list[tuple[int, int]] | None:
    if start == goal:
        return [start]
    if _line_is_clear(start, goal, map_info, offsets):
        return [start, goal]

    wall_mask = map_info.get("wall_mask")
    if wall_mask is None:
        return [start, goal]
    height, width = _wall_mask_shape(wall_mask)
    neighbours = [
        (-stride, 0, stride),
        (stride, 0, stride),
        (0, -stride, stride),
        (0, stride, stride),
        (-stride, -stride, stride * math.sqrt(2.0)),
        (-stride, stride, stride * math.sqrt(2.0)),
        (stride, -stride, stride * math.sqrt(2.0)),
        (stride, stride, stride * math.sqrt(2.0)),
    ]

    heap = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far = {start: 0.0}
    expansions = 0

    while heap and expansions < MAX_ASTAR_EXPANSIONS:
        _, cost, current = heapq.heappop(heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if cost > cost_so_far.get(current, float("inf")):
            continue
        expansions += 1

        for dx, dy, step_cost in neighbours:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                continue
            if not _pixel_is_clear(nxt, map_info, offsets):
                continue
            if dx and dy:
                # Prevent diagonal corner cutting through walls.
                if (not _pixel_is_clear((current[0] + dx, current[1]), map_info, offsets) or
                        not _pixel_is_clear((current[0], current[1] + dy), map_info, offsets)):
                    continue
            new_cost = cost + step_cost
            if new_cost >= cost_so_far.get(nxt, float("inf")):
                continue
            cost_so_far[nxt] = new_cost
            priority = new_cost + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            came_from[nxt] = current
            heapq.heappush(heap, (priority, new_cost, nxt))

    return None


def _smooth_pixel_path(
    path: list[tuple[int, int]],
    map_info: dict,
    offsets: list[tuple[int, int]],
) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_is_clear(path[i], path[j], map_info, offsets):
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed


def _navigation_path(
    start: tuple[float, float],
    target: tuple[float, float],
    map_info: dict | None,
    allow_final_approach: bool = True,
) -> list[tuple[float, float]]:
    if not map_info or map_info.get("wall_mask") is None:
        return [target]

    offsets = _clearance_offsets(map_info)
    stride = _grid_step_px(map_info)
    start_px = _point_to_pixel(start, map_info)
    target_px = _point_to_pixel(target, map_info)
    start_grid = _nearest_clear_grid_pixel(start_px, map_info, offsets, stride)
    goal_grid = _nearest_clear_grid_pixel(target_px, map_info, offsets, stride)
    if start_grid is None or goal_grid is None:
        return [target]

    path = _astar_pixels(start_grid, goal_grid, map_info, offsets, stride)
    if not path:
        return [target]

    smooth = (
        _smooth_pixel_path(path, map_info, offsets)
        if map_info.get("smooth_path", True) else path
    )
    points = [_pixel_to_point(pixel, map_info) for pixel in smooth[1:]]
    if not points:
        points = [_pixel_to_point(goal_grid, map_info)]

    # If the final reported estimate itself is reachable, keep it as the
    # endpoint. If it is very close to a clear approach point, still do a short
    # final approach so the report remains inside the scoring radius.
    last_px = _point_to_pixel(points[-1], map_info)
    if _line_is_clear(last_px, target_px, map_info, offsets):
        points[-1] = target
    elif allow_final_approach and math.hypot(
        points[-1][0] - target[0],
        points[-1][1] - target[1],
    ) <= float(
        map_info.get("final_approach_m", DEFAULT_FINAL_APPROACH_M)
    ):
        points.append(target)
    return points


def plan_route_events(
    all_waypoints: list[tuple[float, float]],
    robot_id: str,
    map_info: dict | None = None,
    reportable_points: set[tuple[float, float]] | None = None,
    preassigned_waypoints: bool = False,
    include_search: bool = True,
) -> list[dict]:
    """
    Return ordered route events for one robot.

    Events with kind ``nav`` are intermediate navigation-only waypoints inserted
    by the optional wall-map planner. ``video`` and ``search`` events should be
    reported when reached; ``sweep`` events are exploratory and should still use
    onboard camera confirmation.
    """
    events: list[dict] = []
    current = START_POS.get(robot_id, (-0.375, 0.0))
    for target in _route_targets(
        all_waypoints,
        robot_id,
        reportable_points,
        preassigned_waypoints=preassigned_waypoints,
        include_search=include_search,
    ):
        original_xy = target["xy"]
        path = _navigation_path(
            current,
            original_xy,
            map_info,
            allow_final_approach=target["kind"] in {"video", "search", "sweep"},
        )
        for nav_point in path[:-1]:
            events.append({"xy": nav_point, "kind": "nav", "report_xy": original_xy})
        final_xy = path[-1] if path else original_xy
        adjusted_m = math.hypot(final_xy[0] - original_xy[0], final_xy[1] - original_xy[1])
        event = dict(target)
        event["xy"] = final_xy
        event["report_xy"] = original_xy
        event["navigation_adjustment_m"] = adjusted_m
        events.append(event)
        current = final_xy
    return events


def plan_route(
    all_waypoints: list[tuple[float, float]],
    robot_id: str,
) -> tuple[list[tuple[float, float]], int, int]:
    """
    Return (route, video_waypoint_count, search_waypoint_count) for one robot.
    """
    events = plan_route_events(all_waypoints, robot_id)
    route = [event["xy"] for event in events]
    n_video = sum(1 for event in events if event["kind"] == "video")
    n_search = sum(1 for event in events if event["kind"] == "search")
    return route, n_video, n_search
