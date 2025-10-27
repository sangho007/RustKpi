#!/usr/bin/env python3
"""Filter map waypoints so only points >= threshold distance remain."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Sequence
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Filter waypoints in a map JSON file so only points at least the given "
            "distance from the previously kept waypoint remain."
        )
    )
    parser.add_argument(
        "map_file",
        help="Path to the source map JSON (e.g. map_data_4lane_measurement.json).",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Destination JSON path (default: <stem>_filtered.json in same directory).",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.01,
        help="Minimum Euclidean distance in same units as the map (default: 0.01).",
    )
    return parser.parse_args()


def load_map(map_path: Path) -> Dict[str, List[Sequence[float]]]:
    with map_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    if not isinstance(data, dict):
        raise ValueError("Map JSON must contain an object at the top level.")
    return data


def euclidean_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def filter_waypoints(
    waypoints: Iterable[Sequence[float]], threshold: float
) -> List[Tuple[float, ...]]:
    filtered: List[Tuple[float, ...]] = []
    for coords in waypoints:
        if not isinstance(coords, Iterable):
            continue
        point = tuple(float(v) for v in coords)
        if not filtered:
            filtered.append(point)
            continue
        if len(point) != len(filtered[-1]):
            continue
        if euclidean_distance(point, filtered[-1]) >= threshold:
            filtered.append(point)
    return filtered


def main() -> None:
    args = parse_args()
    map_path = Path(args.map_file)
    output_path = (
        Path(args.output) if args.output else map_path.with_name(f"{map_path.stem}_filtered.json")
    )

    data = load_map(map_path)
    filtered_data: Dict[str, List[Tuple[float, ...]]] = {}

    for key, waypoints in data.items():
        if not isinstance(waypoints, Iterable):
            continue
        filtered_data[key] = filter_waypoints(waypoints, args.threshold)

    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(filtered_data, handle, ensure_ascii=False, indent=4)

    print(f"Filtered map saved to: {output_path}")
    for key, points in filtered_data.items():
        original_count = len(data.get(key, []))
        print(f"{key}: kept {len(points)} of {original_count} waypoints")


if __name__ == "__main__":
    main()

