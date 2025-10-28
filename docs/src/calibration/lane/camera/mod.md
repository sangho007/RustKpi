# lane/camera/mod.rs — 카메라 입력 캘리브레이션

- 경로: `src/calibration/lane/camera/mod.rs`
- 계층: Calibration / Lane Detection / Camera

## 제공 항목
- `CameraCalibration`: 해상도, 목표 FPS, 캡처 큐 깊이, libcamera 사용 여부, 샘플 영상 경로 등을 보관합니다.
- 보조 메서드 `frame_interval`, `width_u32`, `height_u32`로 시간/해상도 계산을 단순화합니다.

