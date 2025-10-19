# cam_lib.rs — 카메라 캡처 추상화

- 경로: `src/bsw/lib/cam_lib.rs`
- 계층: BSW / 라이브러리

## 목적
카메라 소스별 차이를 추상화하여 동일한 인터페이스로 프레임을 획득할 수 있게 합니다. 현재는 OpenCV `VideoCapture`와 libcamera 브리지 기반 캡처를 지원합니다.

## 공용 인터페이스
- `trait FrameCapture { fn read_frame(&mut self) -> Result<Option<CapturedFrame>>; }`
  - 성공 시 `Some(CapturedFrame)` 반환, EOF/빈 프레임이면 `None`.

## 구현체
- `opencv::videoio::VideoCapture`: OpenCV 표준 API를 그대로 위임.
- `libcamera_capture::LibcameraCapture`:
  - C++ FFI 브리지를 통해 libcamera 파이프라인에서 BGR/GRAY/RGBA 버퍼를 받음.
  - 내부 풀을 사용해 버퍼 재활용 및 `BufferRecycler` 인터페이스 구현.

## 주의/성능
- `CapturedFrame`은 `CameraBuffer`를 `Arc`로 포장해 상위 계층과 버퍼를 공유.
- libcamera 브리지는 stride/bytes-per-pixel을 런타임에 보고하므로 다운스트림에서 해당 값을 신뢰해야 함.
