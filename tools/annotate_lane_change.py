#!/usr/bin/env python3
"""Annotate map waypoints with lane-change availability flags."""

from __future__ import annotations

import argparse
import json
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any, Dict, List


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Add lane-change availability to each waypoint in the map JSON. "
            "Inner lane: x < -1.8 -> cannot change. Outer lane: x < 0.15 -> cannot change."
        )
    )
    parser.add_argument(
        "map_file",
        help="Path to the source map JSON (e.g. map_data_4lane_measurement.json).",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Destination JSON path (default: <stem>_lane_change.json in same directory).",
    )
    return parser.parse_args()


def load_map(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("Map JSON must contain an object at the top level.")
    return data


def waypoint_to_record(
    lane_name: str, waypoint: Sequence[float]
) -> Dict[str, Any]:
    point = [float(value) for value in waypoint]
    if not point:
        raise ValueError(f"Waypoint has no components in lane '{lane_name}'.")

    y_coord = point[1]
    if lane_name.startswith("inner"):
        can_change = not (y_coord >= -0.75 and y_coord <= 0)
    elif lane_name.startswith("outer"):
        can_change = not (y_coord >= -0.75 and y_coord <= 0) 
    else:
        can_change = True

    return {
        "position": point,
        "can_change_lane": can_change,
    }


def annotate_map(data: Dict[str, Any]) -> Dict[str, Any]:
    annotated: Dict[str, Any] = {}
    for key, value in data.items():
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
            records: List[Dict[str, Any]] = []
            for waypoint in value:
                if not isinstance(waypoint, Sequence) or isinstance(waypoint, (str, bytes)):
                    continue
                try:
                    record = waypoint_to_record(key, waypoint)
                except ValueError:
                    continue
                records.append(record)
            annotated[key] = records
        else:
            annotated[key] = value
    return annotated


def main() -> None:
    args = parse_args()
    input_path = Path(args.map_file)
    output_path = (
        Path(args.output)
        if args.output
        else input_path.with_name(f"{input_path.stem}_lane_change.json")
    )

    data = load_map(input_path)
    annotated = annotate_map(data)

    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(annotated, handle, ensure_ascii=False, indent=4)

    print(f"Annotated map saved to: {output_path}")


if __name__ == "__main__":
    main()

