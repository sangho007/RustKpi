#!/usr/bin/env python3

"""이미지 해상도를 원하는 크기로 변경해 저장하는 스크립트."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="입력 이미지를 원하는 해상도로 리사이즈하여 저장합니다."
    )
    parser.add_argument("input", type=Path, help="원본 이미지 경로")
    parser.add_argument(
        "output",
        type=Path,
        help="출력 이미지 경로 (확장자에 따라 저장 포맷이 결정됨)",
    )
    parser.add_argument(
        "--width",
        type=int,
        help="출력 가로 픽셀 수. 생략 시 비율 유지용으로 높이 또는 배율 사용",
    )
    parser.add_argument(
        "--height",
        type=int,
        help="출력 세로 픽셀 수. 생략 시 비율 유지용으로 가로 또는 배율 사용",
    )
    parser.add_argument(
        "--scale",
        type=float,
        help="배율(예: 0.5는 절반, 2는 두 배). width/height와 함께 사용 불가",
    )
    parser.add_argument(
        "--dpi",
        type=float,
        help="옵션: 저장 시 메타데이터에 DPI 값을 기록",
    )
    parser.add_argument(
        "--quality",
        type=int,
        default=95,
        help="JPEG 저장 시 품질(기본 95). 다른 포맷은 무시됨",
    )
    return parser.parse_args()


def resolve_size(
    original_width: int,
    original_height: int,
    width: int | None,
    height: int | None,
    scale: float | None,
) -> tuple[int, int]:
    if scale is not None and (width or height):
        raise ValueError("scale은 width/height와 함께 사용할 수 없습니다.")

    if scale is not None:
        if scale <= 0:
            raise ValueError("scale은 0보다 커야 합니다.")
        new_width = max(1, int(round(original_width * scale)))
        new_height = max(1, int(round(original_height * scale)))
        return new_width, new_height

    if width is None and height is None:
        raise ValueError("width/height 또는 scale 중 하나는 지정해야 합니다.")

    if width is None:
        if height <= 0:
            raise ValueError("height는 0보다 커야 합니다.")
        aspect = original_width / original_height
        width = max(1, int(round(height * aspect)))
    elif width <= 0:
        raise ValueError("width는 0보다 커야 합니다.")

    if height is None:
        aspect = original_height / original_width
        height = max(1, int(round(width * aspect)))
    elif height <= 0:
        raise ValueError("height는 0보다 커야 합니다.")

    return width, height


def main() -> None:
    args = parse_args()

    if not args.input.exists():
        raise FileNotFoundError(f"입력 이미지가 존재하지 않습니다: {args.input}")

    with Image.open(args.input) as img:
        target_size = resolve_size(img.width, img.height, args.width, args.height, args.scale)
        resized = img.resize(target_size, Image.Resampling.LANCZOS)

        save_kwargs = {}
        if args.dpi:
            save_kwargs["dpi"] = (args.dpi, args.dpi)
        if resized.format == "JPEG" or args.output.suffix.lower() in {".jpg", ".jpeg"}:
            save_kwargs["quality"] = args.quality

        resized.save(args.output, **save_kwargs)
        print(f"{args.input} -> {args.output} ({target_size[0]}x{target_size[1]} px)")


if __name__ == "__main__":
    main()
