#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


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


def main() -> None:
    args = parse_args()
    data_path = Path(args.json_path)

    if not data_path.exists():
        raise FileNotFoundError(f"JSON file not found: {data_path}")

    data = load_waypoints(data_path)

    fig, ax = plt.subplots(figsize=(8, 8))

    scatter_waypoints(ax, data.get("inner_waypoint", []), "inner", "#1f77b4")
    scatter_waypoints(ax, data.get("outer_waypoint", []), "outer", "#ff7f0e")

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
