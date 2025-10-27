#!/usr/bin/env python3
"""Hybrid A* path planner and plotter for Rust KPI map JSON files."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt

# Vehicle configuration (meters)
VEHICLE_WIDTH = 0.18
VEHICLE_LENGTH = 0.25
MAX_STEER_RAD = math.radians(30.0)
WHEEL_BASE = 0.20
STEP_SIZE = 0.05
GOAL_POS_TOL = 0.05
GOAL_HEADING_TOL = math.radians(20.0)
GRID_RES = 0.02
HEADING_RES = math.radians(5.0)
MAX_EXPANSIONS = 200000

STEER_OPTIONS = [
    -MAX_STEER_RAD,
    -MAX_STEER_RAD / 2.0,
    0.0,
    MAX_STEER_RAD / 2.0,
    MAX_STEER_RAD,
]


@dataclass
class Node:
    x: float
    y: float
    theta: float
    g: float
    parent: Optional["Node"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute and plot a Hybrid A* path between two waypoints defined by "
            "lane and index."
        )
    )
    parser.add_argument("map_file", help="Map JSON file (e.g. map_data_4lane.json).")
    parser.add_argument(
        "start",
        help="Start lane/index in the form lane,index (e.g. inner,100).",
    )
    parser.add_argument(
        "goal",
        help="Goal lane/index in the form lane,index (e.g. outer,250).",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to save the resulting figure instead of showing it.",
    )
    return parser.parse_args()


def parse_lane_index(spec: str) -> Tuple[str, int]:
    for separator in (",", ":", "/"):
        if separator in spec:
            lane, idx = spec.split(separator, 1)
            break
    else:
        raise ValueError(f"Invalid lane/index specification: {spec}")
    lane = lane.strip().lower()
    index = int(idx.strip())
    return lane, index


def lane_key(lane: str) -> str:
    if lane in {"inner", "inner_waypoint"}:
        return "inner_waypoint"
    if lane in {"outer", "outer_waypoint"}:
        return "outer_waypoint"
    return lane


def normalize_waypoint(item: Any) -> Optional[Tuple[float, float]]:
    if isinstance(item, dict):
        pos = item.get("position")
        if isinstance(pos, Sequence) and len(pos) >= 2:
            return float(pos[0]), float(pos[1])
        return None
    if isinstance(item, Sequence) and not isinstance(item, (str, bytes)) and len(item) >= 2:
        return float(item[0]), float(item[1])
    return None


def extract_waypoints(data: Dict[str, Any], key: str) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    for item in data.get(key, []):
        coords = normalize_waypoint(item)
        if coords is not None:
            points.append(coords)
    return points


def heading_from_index(points: List[Tuple[float, float]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    if index < len(points) - 1:
        nxt = points[index + 1]
        cur = points[index]
    else:
        cur = points[index]
        nxt = points[index - 1]
    dx = nxt[0] - cur[0]
    dy = nxt[1] - cur[1]
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.atan2(dy, dx)


def normalize_angle(theta: float) -> float:
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi
    return theta


def compute_bounds(points: Iterable[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def is_state_valid(
    x: float,
    y: float,
    theta: float,
    bounds: Tuple[float, float, float, float],
) -> bool:
    min_x, max_x, min_y, max_y = bounds
    margin_x = VEHICLE_LENGTH
    margin_y = VEHICLE_WIDTH
    if x < min_x - margin_x or x > max_x + margin_x or y < min_y - margin_y or y > max_y + margin_y:
        return False
    return True


def discretize_state(x: float, y: float, theta: float) -> Tuple[int, int, int]:
    ix = int(round(x / GRID_RES))
    iy = int(round(y / GRID_RES))
    i_theta = int(round(normalize_angle(theta) / HEADING_RES))
    return ix, iy, i_theta


def heuristic(x: float, y: float, theta: float, goal: Node) -> float:
    distance = math.hypot(goal.x - x, goal.y - y)
    heading_error = abs(normalize_angle(goal.theta - theta))
    return distance + 0.1 * heading_error


def forward_model(x: float, y: float, theta: float, steer: float) -> Tuple[float, float, float]:
    dtheta = STEP_SIZE * math.tan(steer) / WHEEL_BASE
    mid_theta = theta + 0.5 * dtheta
    nx = x + STEP_SIZE * math.cos(mid_theta)
    ny = y + STEP_SIZE * math.sin(mid_theta)
    ntheta = normalize_angle(theta + dtheta)
    return nx, ny, ntheta


def hybrid_astar(start: Node, goal: Node, bounds: Tuple[float, float, float, float]) -> List[Node]:
    open_heap: List[Tuple[float, int, Node]] = []
    counter = 0
    start_h = heuristic(start.x, start.y, start.theta, goal)
    heapq.heappush(open_heap, (start_h, counter, start))
    visited: Dict[Tuple[int, int, int], float] = {discretize_state(start.x, start.y, start.theta): 0.0}

    expansions = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        expansions += 1
        if expansions > MAX_EXPANSIONS:
            raise RuntimeError("Reached expansion limit without finding a path.")

        goal_dist = math.hypot(goal.x - current.x, goal.y - current.y)
        heading_diff = abs(normalize_angle(goal.theta - current.theta))
        if goal_dist <= GOAL_POS_TOL and heading_diff <= GOAL_HEADING_TOL:
            return reconstruct_path(current)

        for steer in STEER_OPTIONS:
            nx, ny, ntheta = forward_model(current.x, current.y, current.theta, steer)
            if not is_state_valid(nx, ny, ntheta, bounds):
                continue

            ng = current.g + STEP_SIZE
            key = discretize_state(nx, ny, ntheta)
            if key in visited and visited[key] <= ng:
                continue
            visited[key] = ng

            child = Node(nx, ny, ntheta, ng, current)
            counter += 1
            heapq.heappush(
                open_heap,
                (ng + heuristic(nx, ny, ntheta, goal), counter, child),
            )

    raise RuntimeError("Failed to find a path.")


def reconstruct_path(node: Node) -> List[Node]:
    path: List[Node] = []
    current = node
    while current:
        path.append(current)
        current = current.parent
    return list(reversed(path))


def prepare_state(points: List[Tuple[float, float]], index: int) -> Node:
    if index < 0 or index >= len(points):
        raise IndexError(f"Index {index} out of range for waypoint list of length {len(points)}.")
    position = points[index]
    heading = heading_from_index(points, index)
    return Node(position[0], position[1], heading, 0.0, None)


def plot_environment(
    ax: plt.Axes,
    inner_pts: List[Tuple[float, float]],
    outer_pts: List[Tuple[float, float]],
    path: List[Node],
    start: Node,
    goal: Node,
) -> None:
    if inner_pts:
        xs, ys = zip(*inner_pts)
        ax.plot(xs, ys, "--", color="#1f77b4", linewidth=1.0, label="inner")
    if outer_pts:
        xs, ys = zip(*outer_pts)
        ax.plot(xs, ys, "--", color="#ff7f0e", linewidth=1.0, label="outer")

    if path:
        ax.plot([n.x for n in path], [n.y for n in path], color="#2ca02c", linewidth=2.0, label="path")

    ax.scatter([start.x], [start.y], color="green", s=60, marker="o", label="start")
    ax.scatter([goal.x], [goal.y], color="red", s=60, marker="x", label="goal")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.legend()


def load_map(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def main() -> None:
    args = parse_args()
    map_path = Path(args.map_file)
    if not map_path.exists():
        raise FileNotFoundError(f"Map file not found: {map_path}")

    start_lane, start_index = parse_lane_index(args.start)
    goal_lane, goal_index = parse_lane_index(args.goal)

    data = load_map(map_path)

    inner_points = extract_waypoints(data, lane_key("inner"))
    outer_points = extract_waypoints(data, lane_key("outer"))
    all_points = inner_points + outer_points
    if not all_points:
        raise ValueError("Map JSON does not contain any waypoint data.")
    bounds = compute_bounds(all_points)

    start_points = extract_waypoints(data, lane_key(start_lane))
    goal_points = extract_waypoints(data, lane_key(goal_lane))

    start_state = prepare_state(start_points, start_index)
    goal_state = prepare_state(goal_points, goal_index)

    path_nodes = hybrid_astar(start_state, goal_state, bounds)

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_environment(ax, inner_points, outer_points, path_nodes, start_state, goal_state)

    if args.save:
        fig.savefig(args.save, bbox_inches="tight")
        print(f"Saved figure to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
