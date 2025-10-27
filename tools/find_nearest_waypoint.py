#!/usr/bin/env python3
"""Utility for finding the nearest waypoint in a map JSON file."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Dict, List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Find the nearest waypoint (inner or outer) to a given coordinate."
        )
    )
    parser.add_argument(
        "map_file",
        nargs="?",
        help="Path to the JSON map data (e.g. map_data_4lane_measurement.json).",
    )
    parser.add_argument(
        "--coord",
        nargs="+",
        type=float,
        help="Coordinate to search near, e.g. --coord 1.0 2.0",
    )
    return parser.parse_args()


def prompt_for_missing_args(args: argparse.Namespace) -> Tuple[Path, Tuple[float, ...]]:
    map_path = Path(args.map_file) if args.map_file else Path(
        input("Enter path to map JSON file: ").strip()
    )

    if not args.coord:
        raw_coord = input("Enter coordinate (space separated, e.g. '1.0 2.0'): ").strip()
        coord_values = [float(v) for v in raw_coord.split()]
    else:
        coord_values = args.coord

    if len(coord_values) < 2:
        raise ValueError("Coordinate must have at least two components.")

    return map_path, tuple(coord_values)


def load_map_data(map_path: Path) -> Dict[str, List[Sequence[float]]]:
    if not map_path.exists():
        raise FileNotFoundError(f"Map file not found: {map_path}")

    with map_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError("Map file must contain a JSON object.")

    return data


def compute_distance(point: Sequence[float], candidate: Sequence[float]) -> float:
    # Use Euclidean distance; zip ensures we only compare overlapping dimensions.
    return math.sqrt(sum((p - c) ** 2 for p, c in zip(point, candidate)))


def find_nearest_waypoint(
    map_data: Dict[str, List[Sequence[float]]], point: Sequence[float]
) -> Tuple[str, int, Tuple[float, ...], float]:
    best_lane = ""
    best_index = -1
    best_coords: Tuple[float, ...] = ()
    best_distance = float("inf")

    for lane_name in ("inner_waypoint", "outer_waypoint"):
        waypoints = map_data.get(lane_name, [])
        for idx, coords in enumerate(waypoints):
            if not isinstance(coords, Iterable):
                continue
            coords_tuple = tuple(float(value) for value in coords)
            if len(coords_tuple) != len(point):
                continue
            distance = compute_distance(point, coords_tuple)
            if distance < best_distance:
                best_lane = "inner" if lane_name.startswith("inner") else "outer"
                best_index = idx
                best_coords = coords_tuple
                best_distance = distance

    if best_index == -1:
        raise ValueError("No waypoints found in the provided map data.")

    return best_lane, best_index, best_coords, best_distance


def main() -> None:
    args = parse_args()
    map_path, coord = prompt_for_missing_args(args)
    map_data = load_map_data(map_path)
    lane, index, coords, distance = find_nearest_waypoint(map_data, coord)

    print("Nearest waypoint:")
    print(f"  Lane     : {lane}")
    print(f"  Index    : {index}")
    print(f"  Value    : {coords}")
    print(f"  Distance : {distance:.6f}")


if __name__ == "__main__":
    main()
