# cam_lib.rs — 카메라 캡처 추상화

- 경로: `src/bsw/lib/cam_lib.rs`
- 계층: BSW / 라이브러리

## 목적
카메라 소스별 차이를 추상화하여 동일한 인터페이스로 프레임을 획득할 수 있게 합니다. OpenCV `VideoCapture`와 Python `PiCamera2`를 모두 지원합니다.

## 공용 인터페이스
- `trait FrameCapture { fn read_frame(&mut self, frame: &mut Mat) -> Result<bool>; }`
  - 성공 시 `true`, EOF/빈 프레임 시 `false` 반환.

## 구현체
- `opencv::videoio::VideoCapture`: OpenCV 표준 API를 그대로 위임.
- `picamera_capture::PiCamera2`:
  - `pyo3`로 Python `Picamera2`를 호출, BGR888 NumPy 배열을 OpenCV `Mat`으로 zero‑copy view 후 deep clone.
  - `Drop`에서 `stop()` 호출로 장치 정지.
  - 실패 시 메시지 기반으로 OpenCV `Error`로 변환.

## 주의/성능
- NumPy → Mat 변환은 안전을 위해 최종 deep clone 사용.
- BGR888 포맷 사용으로 OpenCV 파이프라인과 정합.

