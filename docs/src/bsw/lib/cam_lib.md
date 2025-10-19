# cam_lib.rs - 카메라 캡처 추상화

- 경로: `src/bsw/lib/cam_lib.rs`
- 계층: BSW / 라이브러리

## 핵심 개념
- `CapturedFrame`: `CameraBuffer`(픽셀 데이터), 해상도, stride, bytes_per_pixel, `ColorFormat` 메타데이터를 포함한 구조체. `Arc`로 공유해 다운스트림 복사를 피합니다.
- `CameraBuffer`: 선택적으로 `BufferRecycler`를 붙여 libcamera 풀 메모리를 재활용할 수 있는 버퍼 래퍼.
- `FrameCapture` 트레이트: `read_frame(&mut self) -> Result<Option<CapturedFrame>>` 인터페이스로 다양한 캡처 백엔드를 통일합니다.

## 구현체
- `videoio::VideoCapture`:
  - OpenCV 캡처에서 프레임을 읽어 `fit_frame_to_target`으로 캘리브레이션된 해상도에 맞춥니다.
  - stride/bytes_per_pixel을 계산해 `CameraBuffer::from_vec`에 저장합니다.
- `libcamera_capture::LibcameraCapture`:
  - C++ 브리지(`libcamera_bridge.cpp`)를 통해 큐잉된 버퍼를 받아오고, 재활용 풀(`BufferPool`)로 메모리를 재사용합니다.
  - `CapturedFrame::new`에 stride, bytes_per_pixel, `ColorFormat`을 함께 전달합니다.

## 보조 함수
- `fit_frame_to_target(frame: &Mat) -> Result<Mat>`: 입력 영상 비율에 따라 크롭 또는 레터박스로 조정 후 `CameraCalibration::default()`가 정의한 폭/높이로 리사이즈합니다.
- `mat_from_buffer` / `ensure_bgr`: RTE DTO에서 `CameraBuffer`를 OpenCV `Mat`으로 바라보거나 BGR 색 공간으로 변환할 때 사용합니다.

## 활용
- `bsw::ecu_abs_cam::ea_cam_provider`는 백엔드(libcamera/비디오 파일)를 추상화하기 위해 `FrameCapture` 트레이트와 `CapturedFrame` 메타데이터를 사용합니다.
- ASW 단계는 `DtoCamRaw`에서 stride와 색 포맷 정보를 참조해 안전하게 `Mat` 뷰를 생성합니다.
