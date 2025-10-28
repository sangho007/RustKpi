# docker/dockerfile — 멀티스테이지 빌드 환경

- 경로: `docker/dockerfile`
- 계층: Docker

## 스테이지 개요
- **builder**: Ubuntu 24.04 기반으로 OpenCV 4.11, libcamera(next), rpicam-apps, kmsxx 등 카메라 스택을 직접 빌드하고 Rust toolchain을 설치합니다.
- **final**: 런타임에 필요한 라이브러리와 Python 패키지만 포함한 경량 이미지로, builder 스테이지에서 생성한 결과물을 복사합니다.

## 특징
- libcamera 헤더 충돌을 피하기 위해 apt `libcamera-dev` 패키지를 설치하지 않고 소스에서 빌드합니다.
- LLVM 17/Clang 17을 설치해 Rust의 FFI 빌드 시 최신 툴체인을 사용합니다.
- Raspberry Pi(ARM64) 환경을 염두에 둔 패키지 목록과 환경 변수(`PKG_CONFIG_PATH`, `LD_LIBRARY_PATH`)를 설정합니다.

