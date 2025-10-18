# vs_lane_lib.rs — 차선 인식 파이프라인 라이브러리

- 경로: `src/asw/lib/vs_lane_lib.rs`
- 계층: ASW / 라이브러리 (Vision)

## 목적
OpenCV 기반 차선 인식 파이프라인의 핵심 로직을 제공합니다. 전처리, ROI/투시 변환, 슬라이딩 윈도우 탐색, 곡선 피팅, 조향각 계산, 칼만 필터 평활화까지 포함합니다.

## 핵심 타입
- `Pipeline` 구조체: 해상도, ROI, 변환행렬, 윈도우 파라미터, 최근 곡선 계수(좌/우), 플롯용 y, 조향각 상태, 칼만 필터 파라미터 포함.
- `LaneTaskConfig { use_kalman }`

## 주요 메서드
- 생성/설정: `new()`, `new_with_settings(use_kalman)`
- 전처리: `gray_scale`, `noise_removal(gaussian_blur)`, `edge_detection(canny)`, `morphology_close`
- 영역/투시: `roi`, `perspective_transform`, `inv_perspective_transform`
- 탐색:
  - `sliding_window(binary, nwindows, margin, minpix, draw)`
  - `search_around_poly(binary, margin)` (과거 곡선 주변 빠른 추적)
- 조향각: `get_angle_on_lane(left_fitx, right_fitx, left_detected, right_detected)`
- 평활화: `update_angle_kalman(measurement)`, `reset_kalman(init, P0)`
- 시각화/유틸: `display_heading_line`, `hconcat_2`

## 수학/헬퍼
- 곡선 피팅: `polyfit_1d`, `polyfit_2d` (최소제곱/크래머 룰)
- 평균화: `mean_of_last_10`
- 병렬화: `get_nonzero_points_by_row`에 Rayon 적용(행 단위 병렬 처리)

## 성능/설계 포인트
- ROI와 투시 변환으로 탐색 공간 축소 → 속도 향상
- 과거 계수의 이동 평균으로 안정화, 실패 시 기준점 리셋 후 재탐색
- 필요 시 칼만 필터로 각도 시계열 평활화

## 반환/에러
- OpenCV `Result<T>` 별칭(`LaneDetectionResult<T>`) 사용, `?` 전파로 간결한 에러 처리

