# build.rs — libcamera 브릿지 빌드 스크립트

- 경로: `build.rs`
- 계층: 루트 빌드 단계

## 역할
- `cc::Build`를 사용해 `src/bsw/lib/libcamera_bridge.cpp`를 C++17로 컴파일하고, `pkg-config`로 발견한 `libcamera` 헤더/라이브러리를 링크합니다.
- 다양한 include 경로를 dedupe하여 등록하고, 라즈베리파이 환경을 고려한 폴백 경로(`/usr/include/libcamera` 등)를 추가합니다.
- `LIBCAMERA_BRIDGE_CXXFLAGS` 환경 변수로 추가 플래그를 주입할 수 있으며, 관련 파일 변경 시 Cargo 빌드를 다시 실행하도록 트리거합니다.

