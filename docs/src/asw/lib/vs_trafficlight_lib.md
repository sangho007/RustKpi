# vs_trafficlight_lib.rs - 신호등 인지 파이프라인

- 경로: `src/asw/lib/vs_trafficlight_lib.rs`
- 계층: ASW / 라이브러리 (Vision)

## 핵심 타입
- `TrafficLightColor` 열거형: `Red`, `Yellow`, `Green`, `Off`
- `Pipeline`: ROI 다각형, 색상별 HSV 임계값, 모폴로지/DBSCAN 파라미터, 최근 판정 상태를 보존합니다. `TrafficLightCalibration`을 입력으로 생성됩니다.

## 주요 메서드
- `Pipeline::new(calibration)`: ROI 정점과 HSV 임계값, DBSCAN 설정, 최소 픽셀 임계값을 캘리브레이션에서 복사해 초기화합니다.
- `convert_to_hsv(&Mat) -> Result<Mat>`: BGR 프레임을 HSV로 변환.
- `detect_color_from_hsv(&mut self, &Mat) -> TrafficLightColor`:
  1. 각 색상 범위로 `create_mask` 실행.
  2. `apply_morphology`(타원 커널, opening)로 노이즈 제거.
  3. `find_largest_cluster`로 DBSCAN을 적용해 최대 클러스터 크기를 계산.
  4. 최소 픽셀 임계값을 만족하고 가장 큰 클러스터를 가진 색상을 선택, 없으면 `Off`.
- `create_mask`: HSV 하한/상한으로 1채널 마스크를 생성.
- `apply_morphology`: `MORPH_OPEN`을 1회 적용해 노이즈 제거.
- `find_largest_cluster`: 비영점 픽셀 좌표를 DBSCAN에 입력해 클러스터별 크기를 구하고 최댓값을 반환.

## 튜닝 포인트
- `TrafficLightCalibration`에서 ROI, HSV 범위, `min_pixel_threshold`, `dbscan_epsilon`, `dbscan_min_points`를 조정해 환경에 맞출 수 있습니다.
- 검출이 빈번히 `Off`로 떨어지면 최소 픽셀 수를 낮추거나 DBSCAN 파라미터를 완화합니다.
