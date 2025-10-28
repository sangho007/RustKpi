# traffic_light.rs — 신호등 인식 캘리브레이션

- 경로: `src/calibration/traffic_light.rs`
- 계층: Calibration / Vision

## 주요 요소
- HSV 색상 임계값(`TrafficLightColorThreshold`)으로 빨강/노랑/초록을 구분합니다.
- ROI 좌표를 카메라 해상도에 맞춰 스케일링하고, DBSCAN 클러스터링 파라미터를 조정해 노이즈를 제거합니다.

