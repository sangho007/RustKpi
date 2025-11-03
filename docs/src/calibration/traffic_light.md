# traffic_light.rs — 신호등 인식 캘리브레이션

- 경로: `src/calibration/traffic_light.rs`
- 계층: Calibration / Vision

## 목적
- 신호등 인식 파이프라인이 사용할 ROI와 HSV 임계값을 중앙에서 관리해 환경별 튜닝을 단순화합니다.
- 노이즈 제거와 클러스터링 파라미터를 명시해 안정적인 색상 판정을 지원합니다.

## 제공 항목
- HSV 색상 임계값(`TrafficLightColorThreshold`)으로 빨강/노랑/초록을 구분합니다.
- ROI 좌표를 카메라 해상도에 맞춰 스케일링하고, DBSCAN 클러스터링 파라미터를 조정해 노이즈를 제거합니다.
