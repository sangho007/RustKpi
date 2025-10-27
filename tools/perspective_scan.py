#!/usr/bin/env python3
"""
Interactive perspective correction tool.

Left-click four corners of the document in the displayed window and confirm to
warp the image into a rectangle, similar to a mobile scanning app.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


Point = Tuple[int, int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select four points on an image and save a perspective-corrected "
            "rectangular output."
        )
    )
    parser.add_argument("input", help="Path to the source image")
    parser.add_argument("output", help="Path for the perspective-corrected image")
    parser.add_argument(
        "--width",
        type=int,
        default=None,
        help="Force output width (pixels). Height keeps aspect ratio unless --height is given.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=None,
        help="Force output height (pixels). Width keeps aspect ratio unless --width is given.",
    )
    parser.add_argument(
        "--window-title",
        default="Select 4 corners (CCW)",
        help="Title for the OpenCV window",
    )
    return parser.parse_args()


def select_quadrilateral(image: np.ndarray, window_title: str) -> Optional[List[Point]]:
    points: List[Point] = []
    display = image.copy()

    def on_mouse(event, x, y, *_):
        nonlocal points
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 4:
                points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            if points:
                points.pop()

    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_title, on_mouse)

    instructions = (
        "Left click: add corner  |  Right click: undo  |  R: reset  |  Enter: confirm  |  Q: quit"
    )

    try:
        while True:
            display[:] = image
            for idx, (x, y) in enumerate(points, start=1):
                cv2.circle(display, (x, y), 6, (0, 255, 0), -1)
                cv2.putText(
                    display,
                    str(idx),
                    (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            if len(points) == 4:
                cv2.polylines(
                    display,
                    [np.array(points, dtype=np.int32)],
                    isClosed=True,
                    color=(255, 0, 0),
                    thickness=2,
                )

            cv2.rectangle(display, (10, 10), (display.shape[1] - 10, 45), (0, 0, 0), -1)
            cv2.putText(
                display,
                instructions,
                (16, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(window_title, display)
            key = cv2.waitKey(20) & 0xFF

            if key in (ord("q"), 27):  # 'q' or ESC
                return None
            if key in (ord("r"), ord("R")):
                points.clear()
            if key in (13, 10, 32) and len(points) == 4:  # Enter, newline, or space
                break
    finally:
        cv2.destroyWindow(window_title)

    return points if len(points) == 4 else None


def order_points(pts: List[Point]) -> np.ndarray:
    rect = np.array(pts, dtype="float32")
    s = rect.sum(axis=1)
    diff = np.diff(rect, axis=1)

    top_left = rect[np.argmin(s)]
    bottom_right = rect[np.argmax(s)]
    top_right = rect[np.argmin(diff)]
    bottom_left = rect[np.argmax(diff)]

    return np.array([top_left, top_right, bottom_right, bottom_left], dtype="float32")


def compute_target_size(
    rect: np.ndarray, width_hint: Optional[int], height_hint: Optional[int]
) -> Tuple[int, int]:
    (top_left, top_right, bottom_right, bottom_left) = rect
    width_top = np.linalg.norm(top_right - top_left)
    width_bottom = np.linalg.norm(bottom_right - bottom_left)
    height_right = np.linalg.norm(bottom_right - top_right)
    height_left = np.linalg.norm(bottom_left - top_left)

    natural_width = max(width_top, width_bottom)
    natural_height = max(height_right, height_left)

    if natural_width <= 0 or natural_height <= 0:
        raise ValueError("Selected points are degenerate; cannot compute size.")

    aspect = natural_width / natural_height

    if width_hint and height_hint:
        return width_hint, height_hint
    if width_hint:
        return width_hint, max(1, int(round(width_hint / aspect)))
    if height_hint:
        return max(1, int(round(height_hint * aspect))), height_hint

    return int(round(natural_width)), int(round(natural_height))


def warp_perspective(
    image: np.ndarray,
    points: List[Point],
    width_hint: Optional[int],
    height_hint: Optional[int],
) -> Tuple[np.ndarray, int, int]:
    rect = order_points(points)
    target_width, target_height = compute_target_size(rect, width_hint, height_hint)

    dst = np.array(
        [
            [0, 0],
            [target_width - 1, 0],
            [target_width - 1, target_height - 1],
            [0, target_height - 1],
        ],
        dtype="float32",
    )

    transform = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, transform, (target_width, target_height))
    return warped, target_width, target_height


def main() -> None:
    args = parse_args()
    src_path = Path(args.input)
    if not src_path.is_file():
        print(f"Input file not found: {src_path}", file=sys.stderr)
        sys.exit(1)

    image = cv2.imread(str(src_path))
    if image is None:
        print(f"Failed to read image: {src_path}", file=sys.stderr)
        sys.exit(1)

    points = select_quadrilateral(image, args.window_title)
    if points is None:
        print("Selection aborted.", file=sys.stderr)
        sys.exit(1)

    try:
        warped, width, height = warp_perspective(
            image, points, args.width, args.height
        )
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not cv2.imwrite(str(output_path), warped):
        print(f"Failed to save image: {output_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Saved rectified image to {output_path} ({width}x{height}).")


if __name__ == "__main__":
    main()
