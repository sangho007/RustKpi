#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from collections.abc import Iterable, Sequence


def normalize_waypoint(item: object) -> tuple[float, float] | None:
    if isinstance(item, dict):
        position = item.get("position")
        if isinstance(position, Sequence) and len(position) >= 2:
            return float(position[0]), float(position[1])
        return None
    if isinstance(item, Sequence) and not isinstance(item, (str, bytes)) and len(item) >= 2:
        return float(item[0]), float(item[1])
    return None


def is_lane_change_allowed(item: object) -> bool:
    if isinstance(item, dict):
        return bool(item.get("can_change_lane", True))
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scatter plot for map_data_4lane.json waypoints."
    )
    parser.add_argument(
        "json_path",
        nargs="?",
        default="map_data_4lane.json",
        help="Path to the map data JSON file (default: map_data_4lane.json).",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to save the figure instead of showing it.",
    )
    return parser.parse_args()


def load_waypoints(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def scatter_waypoints(ax: plt.Axes, points: list, label: str, color: str) -> None:
    if not points:
        return
    xs, ys = zip(*points)
    ax.scatter(xs, ys, s=10, label=label, color=color)


def split_waypoints(points: Iterable[object]) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    allowed: list[tuple[float, float]] = []
    blocked: list[tuple[float, float]] = []
    for item in points or []:
        coords = normalize_waypoint(item)
        if coords is None:
            continue
        if is_lane_change_allowed(item):
            allowed.append(coords)
        else:
            blocked.append(coords)
    return allowed, blocked


def main() -> None:
    args = parse_args()
    data_path = Path(args.json_path)

    if not data_path.exists():
        raise FileNotFoundError(f"JSON file not found: {data_path}")

    data = load_waypoints(data_path)

    fig, ax = plt.subplots(figsize=(8, 8))

    inner_allowed, inner_blocked = split_waypoints(data.get("inner_waypoint", []))
    outer_allowed, outer_blocked = split_waypoints(data.get("outer_waypoint", []))

    scatter_waypoints(ax, inner_allowed, "inner (change OK)", "#1f77b4")
    scatter_waypoints(ax, inner_blocked, "inner (change blocked)", "#d62728")
    scatter_waypoints(ax, outer_allowed, "outer (change OK)", "#ff7f0e")
    scatter_waypoints(ax, outer_blocked, "outer (change blocked)", "#2ca02c")

    ax.set_title(data_path.name)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.axis("equal")
    ax.grid(True, linestyle="--", linewidth=0.5)
    ax.legend()

    if args.save:
        fig.savefig(args.save, bbox_inches="tight")
    else:
        plt.show()


if __name__ == "__main__":
    main()
