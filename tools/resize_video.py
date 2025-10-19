#!/usr/bin/env python3
"""
Utility script to create a 640x480 version of the sample video while preserving aspect ratio.

Usage:
    python tools/resize_video.py --input video/challenge.mp4 --output video/challenge_640x480.mp4
"""

import argparse
import pathlib

import cv2


TARGET_WIDTH = 640
TARGET_HEIGHT = 480
TARGET_RATIO = TARGET_WIDTH / TARGET_HEIGHT


def crop_to_ratio(frame):
    height, width = frame.shape[:2]
    if height == 0 or width == 0:
        return frame

    src_ratio = width / height
    if abs(src_ratio - TARGET_RATIO) < 1e-6:
        return frame

    if src_ratio > TARGET_RATIO:
        new_width = int(round(height * TARGET_RATIO))
        offset = (width - new_width) // 2
        return frame[:, offset : offset + new_width]

    new_height = int(round(width / TARGET_RATIO))
    offset = (height - new_height) // 2
    return frame[offset : offset + new_height, :]


def convert(input_path: pathlib.Path, output_path: pathlib.Path):
    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open input video: {input_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(
        str(output_path), fourcc, fps, (TARGET_WIDTH, TARGET_HEIGHT), True
    )
    if not out.isOpened():
        raise RuntimeError(f"Failed to open output video: {output_path}")

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = crop_to_ratio(frame)
        frame = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_LINEAR)
        out.write(frame)
        frame_count += 1

    cap.release()
    out.release()
    print(f"Wrote {frame_count} frames to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Resize sample video to 640x480.")
    parser.add_argument("--input", type=pathlib.Path, required=True, help="Path to source video.")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=pathlib.Path("video/challenge_640x480.mp4"),
        help="Path to write resized video.",
    )
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    convert(args.input, args.output)


if __name__ == "__main__":
    main()
