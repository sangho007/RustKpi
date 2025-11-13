#!/usr/bin/env python3
"""
그레이스케일 변환 도구

사용법:
  python3 tools/convert_grayscale.py <입력이미지> <출력이미지> [--method pil|opencv] [--keep-alpha]

기본은 PIL(Pillow)로 8비트 그레이스케일(L)로 변환합니다. --keep-alpha를 주면 원본의 알파를 유지한 LA로 저장합니다.
OpenCV가 설치되어 있고 --method opencv를 지정하면 OpenCV로 변환합니다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="입력 이미지를 그레이스케일로 변환합니다.")
    p.add_argument("input", type=Path, help="입력 이미지 경로")
    p.add_argument("output", type=Path, help="출력 이미지 경로")
    p.add_argument(
        "--method",
        choices=["pil", "opencv"],
        default="pil",
        help="변환 라이브러리 선택(기본 pil)",
    )
    p.add_argument(
        "--keep-alpha",
        action="store_true",
        help="PIL 사용 시 원본 알파 채널을 유지(LA) 저장",
    )
    return p.parse_args()


def convert_with_pil(src: Path, dst: Path, keep_alpha: bool) -> None:
    from PIL import Image

    with Image.open(src) as im:
        has_alpha = im.mode in ("LA", "RGBA", "PA") or ("transparency" in im.info)
        if keep_alpha and has_alpha:
            rgb = im.convert("RGBA")
            gray = rgb.convert("L")
            out = Image.merge("LA", (gray, rgb.getchannel("A")))
        else:
            out = im.convert("L")

        dst.parent.mkdir(parents=True, exist_ok=True)
        save_kwargs = {}
        # JPEG 품질 기본 95
        if dst.suffix.lower() in {".jpg", ".jpeg"}:
            save_kwargs["quality"] = 95
        out.save(dst, **save_kwargs)


def convert_with_opencv(src: Path, dst: Path) -> None:
    import cv2

    img = cv2.imread(str(src), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {src}")

    # IMREAD_UNCHANGED로 불러오면 채널 수가 다양할 수 있음.
    if img.ndim == 2:
        gray = img  # 이미 그레이스케일
    elif img.shape[2] == 4:
        bgr = img[:, :, :3]
        alpha = img[:, :, 3]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        # OpenCV는 알파 포함 그레이(Single + alpha) 저장을 직접 지원하지 않으니 PNG일 때만 합성
        if dst.suffix.lower() == ".png":
            import numpy as np

            gray = cv2.merge([gray, alpha])  # GA 2채널
        # 그 외 포맷은 알파를 버리고 단일 채널로 저장
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dst.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(dst), gray)
    if not ok:
        raise RuntimeError(f"이미지 저장 실패: {dst}")


def main() -> None:
    args = parse_args()
    if not args.input.exists():
        raise FileNotFoundError(f"입력 이미지가 존재하지 않습니다: {args.input}")

    if args.method == "opencv":
        try:
            convert_with_opencv(args.input, args.output)
        except ImportError:
            raise SystemExit("OpenCV가 설치되어 있지 않습니다. pip install opencv-python 또는 --method pil 사용")
    else:
        try:
            convert_with_pil(args.input, args.output, args.keep_alpha)
        except ImportError:
            raise SystemExit("Pillow가 설치되어 있지 않습니다. pip install Pillow 또는 --method opencv 사용")

    print(f"{args.input} -> {args.output} (grayscale)")


if __name__ == "__main__":
    main()

