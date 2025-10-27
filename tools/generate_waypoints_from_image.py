#!/usr/bin/env python3

"""Extract inner/outer waypoints from a colored reference image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Callable, Iterable, List, Optional

import cv2
import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate waypoint JSON data from a color-coded reference image."
    )
    parser.add_argument(
        "--image",
        type=Path,
        default=Path("1자_wp.jpg"),
        help="Input image with red inner and blue outer waypoints (default: 1자_wp.jpg).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("map_data_1lane_measurement.json"),
        help="Destination JSON file (default: map_data_1lane_measurement.json).",
    )
    parser.add_argument(
        "--pixels-per-cm",
        type=float,
        default=20.0,
        help="Number of pixels that represent one centimeter (default: 20.0).",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Keep every N-th point along the path to down-sample (default: 1).",
    )
    parser.add_argument(
        "--min-component",
        type=int,
        default=100,
        help="Minimum skeleton component size (in pixels) to keep (default: 100).",
    )
    return parser.parse_args()


def load_image(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Image not found: {path}")
    with Image.open(path) as img:
        return np.asarray(img.convert("RGB"))


def make_red_mask(arr: np.ndarray) -> np.ndarray:
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    return (r >= 180) & (g <= 120) & (b <= 120)


def make_blue_mask(arr: np.ndarray) -> np.ndarray:
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    return (b >= 200) & (r <= 80) & (g >= 80) & (g <= 180)


def _shift(img: np.ndarray, dy: int, dx: int) -> np.ndarray:
    shifted = np.roll(img, dy, axis=0)
    shifted = np.roll(shifted, dx, axis=1)
    if dy > 0:
        shifted[:dy, :] = 0
    elif dy < 0:
        shifted[dy:, :] = 0
    if dx > 0:
        shifted[:, :dx] = 0
    elif dx < 0:
        shifted[:, dx:] = 0
    return shifted


def _thinning_iteration(img: np.ndarray, iteration: int) -> np.ndarray:
    P2 = _shift(img, -1, 0)
    P3 = _shift(img, -1, 1)
    P4 = _shift(img, 0, 1)
    P5 = _shift(img, 1, 1)
    P6 = _shift(img, 1, 0)
    P7 = _shift(img, 1, -1)
    P8 = _shift(img, 0, -1)
    P9 = _shift(img, -1, -1)

    neighbors = [P2, P3, P4, P5, P6, P7, P8, P9]
    B = sum(neighbors)

    transitions = ((P2 == 0) & (P3 == 1)).astype(np.uint8)
    transitions += ((P3 == 0) & (P4 == 1)).astype(np.uint8)
    transitions += ((P4 == 0) & (P5 == 1)).astype(np.uint8)
    transitions += ((P5 == 0) & (P6 == 1)).astype(np.uint8)
    transitions += ((P6 == 0) & (P7 == 1)).astype(np.uint8)
    transitions += ((P7 == 0) & (P8 == 1)).astype(np.uint8)
    transitions += ((P8 == 0) & (P9 == 1)).astype(np.uint8)
    transitions += ((P9 == 0) & (P2 == 1)).astype(np.uint8)

    marker = (img == 1) & (B >= 2) & (B <= 6) & (transitions == 1)

    if iteration == 0:
        marker &= (P2 * P4 * P6 == 0) & (P4 * P6 * P8 == 0)
    else:
        marker &= (P2 * P4 * P8 == 0) & (P2 * P6 * P8 == 0)

    return np.where(marker, 0, img)


def zhang_suen_thinning(mask: np.ndarray) -> np.ndarray:
    img = mask.astype(np.uint8)
    prev = np.zeros_like(img)

    while True:
        img = _thinning_iteration(img, 0)
        img = _thinning_iteration(img, 1)

        if np.array_equal(img, prev):
            break

        prev = img.copy()

    return img


def preprocess_mask(mask: np.ndarray) -> np.ndarray:
    mask_u8 = (mask.astype(np.uint8) * 255)
    mask_u8 = cv2.medianBlur(mask_u8, 3)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel, iterations=1)
    return (mask_u8 > 0).astype(np.uint8)


def skeletonize_mask(mask: np.ndarray, min_component: int) -> np.ndarray:
    cleaned = preprocess_mask(mask)
    skeleton = zhang_suen_thinning(cleaned)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        skeleton.astype(np.uint8), connectivity=8
    )

    filtered = np.zeros_like(skeleton)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] >= min_component:
            filtered[labels == label] = 1

    return filtered


def mask_to_paths(
    mask: np.ndarray,
    stride: int,
    min_component: int,
) -> List[np.ndarray]:
    skeleton = skeletonize_mask(mask, min_component)
    if not skeleton.any():
        return []

    contours, _ = cv2.findContours(
        (skeleton * 255).astype(np.uint8),
        mode=cv2.RETR_LIST,
        method=cv2.CHAIN_APPROX_NONE,
    )

    paths: List[np.ndarray] = []
    for contour in contours:
        if contour.shape[0] < 2:
            continue

        pts = contour[:, 0, :]
        if pts.shape[0] > 1 and np.array_equal(pts[0], pts[-1]):
            pts = pts[:-1]

        if stride > 1:
            pts = pts[::stride]

        if pts.size == 0:
            continue

        paths.append(pts)

    paths.sort(key=lambda arr: (arr[:, 1].min(), arr[:, 0].min()))
    return paths


def _dedupe_consecutive(points: Iterable[tuple[int, int]]) -> list[tuple[int, int]]:
    deduped: list[tuple[int, int]] = []
    for row, col in points:
        if not deduped or deduped[-1] != (row, col):
            deduped.append((row, col))
    return deduped


def _rotate_path_to_anchor(
    points: list[tuple[int, int]],
    anchor: tuple[int, int],
) -> list[tuple[int, int]]:
    if not points:
        return points

    best_sequence = points
    best_score = float("inf")

    for reverse in (False, True):
        seq = points[::-1] if reverse else points
        distances = [
            ((row - anchor[0]) ** 2 + (col - anchor[1]) ** 2, idx)
            for idx, (row, col) in enumerate(seq)
        ]
        score, best_idx = min(distances, key=lambda item: item[0])
        if score < best_score:
            best_score = score
            best_sequence = seq[best_idx:] + seq[:best_idx]

    return best_sequence


def pixels_to_meter_points(
    pixel_paths: Iterable[np.ndarray],
    width: int,
    height: int,
    meters_per_pixel: float,
) -> list[list[float]]:
    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0

    waypoints: list[list[float]] = []
    anchor: Optional[tuple[int, int]] = None
    for pts in pixel_paths:
        if pts.shape[0] == 0:
            continue

        coords = _dedupe_consecutive((int(row), int(col)) for col, row in pts)
        if not coords:
            continue

        if anchor is not None:
            coords = _rotate_path_to_anchor(coords, anchor)

        for row_i, col_i in coords:
            x_m = (col_i - center_x) * meters_per_pixel
            y_m = (center_y - row_i) * meters_per_pixel
            waypoints.append([round(x_m, 6), round(y_m, 6)])

        anchor = coords[-1]

    return waypoints


def extract_waypoints(
    mask_fn: Callable[[np.ndarray], np.ndarray],
    image: np.ndarray,
    meters_per_pixel: float,
    stride: int,
    min_component: int,
) -> list[list[float]]:
    mask = mask_fn(image)
    paths = mask_to_paths(mask.astype(np.uint8), stride=stride, min_component=min_component)
    if not paths:
        return []

    points = pixels_to_meter_points(
        pixel_paths=paths,
        width=image.shape[1],
        height=image.shape[0],
        meters_per_pixel=meters_per_pixel,
    )
    return points


def main() -> None:
    args = parse_args()

    if args.pixels_per_cm <= 0:
        raise ValueError("pixels-per-cm must be positive.")
    if args.stride <= 0:
        raise ValueError("stride must be positive.")
    if args.min_component <= 0:
        raise ValueError("min-component must be positive.")

    image = load_image(args.image)
    meters_per_pixel = 1.0 / (args.pixels_per_cm * 100.0)

    inner_waypoints = extract_waypoints(
        make_red_mask,
        image,
        meters_per_pixel,
        args.stride,
        args.min_component,
    )
    outer_waypoints = extract_waypoints(
        make_blue_mask,
        image,
        meters_per_pixel,
        args.stride,
        args.min_component,
    )

    if not inner_waypoints:
        raise RuntimeError("No inner (red) waypoints found in the image.")
    if not outer_waypoints:
        raise RuntimeError("No outer (blue) waypoints found in the image.")

    payload = {
        "inner_waypoint": inner_waypoints,
        "outer_waypoint": outer_waypoints,
    }

    args.output.write_text(json.dumps(payload, indent=4), encoding="utf-8")
    print(
        f"Wrote {len(inner_waypoints)} inner and {len(outer_waypoints)} outer waypoints to {args.output}"
    )


if __name__ == "__main__":
    main()
