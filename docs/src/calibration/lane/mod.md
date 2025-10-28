# lane/mod.rs — 차선 파이프라인 캘리브레이션 집합

- 경로: `src/calibration/lane/mod.rs`
- 계층: Calibration / Lane Detection

## 개요
- 카메라 입력, 이미지 전처리, 슬라이딩 윈도우, 칼만 필터, 런타임 스케줄링을 묶은 `LaneCalibration` 구조체를 제공합니다.
- `LaneCalibrationPreset`으로 HD(1280x720)와 VGA(640x480) 프리셋을 준비해 해상도에 맞는 설정을 빠르게 선택할 수 있습니다.

## 특징
- HD 프리셋은 기본 ROI/투시 좌표를 비율에 맞게 스케일링합니다.
- VGA 프리셋은 `SlidingWindowCalibration`을 변경해 좁은 해상도에서도 견고하게 동작하도록 조정합니다.

