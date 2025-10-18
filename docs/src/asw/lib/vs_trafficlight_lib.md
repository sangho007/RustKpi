# vs_trafficlight_lib.rs — 신호등 인지 라이브러리

- 경로: `src/asw/lib/vs_trafficlight_lib.rs`
- 계층: ASW / 라이브러리 (Vision)

## 목적
HSV 색 공간 기반의 색상 마스크 생성, 모폴로지 노이즈 제거, DBSCAN으로 가장 큰 색 영역(클러스터) 추정 후 신호등 상태를 판별합니다.

## 핵심 타입/상수
- `TrafficLightColor { Red, Yellow, Green, Off }`
- `Pipeline { width, height, vertices, red/yellow/green_threshold, current_traffic_light_color }`
- 클러스터 최소 픽셀: `MIN_PIXEL_THRESHOLD = 100`
- DBSCAN: `EPSILON = 20.0`, `MIN_POINTS = 15`

## 주요 메서드
- `convert_to_hsv(&Mat) -> Result<Mat>`
- `detect_color_from_hsv(&mut Mat) -> TrafficLightColor`
  - 내부: `create_mask` → `apply_morphology`(열림) → `find_largest_cluster`
- `create_mask(hsv, (lower,upper))` — HSV 범위 필터
- `apply_morphology(mask)` — 노이즈 제거(타원 커널)
- `find_largest_cluster(mask)` — 비영점 픽셀에 DBSCAN 적용 후 최대 클러스터 크기 반환

## 튜닝 포인트
- HSV 임계값 범위, 모폴로지 커널 크기, DBSCAN 파라미터(EPS/MIN_POINTS)
- `MIN_PIXEL_THRESHOLD`로 오탐/미탐 균형 조정

