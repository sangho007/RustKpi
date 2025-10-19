# asw/lib/mod.rs - ASW 라이브러리 인덱스

- 경로: `src/asw/lib/mod.rs`
- 포함 모듈: `forwardcollision_ultrasonic_lib`, `vs_lane_lib`, `vs_trafficlight_lib`

ASW 단계에서 공통으로 사용하는 보조 로직을 제공합니다.
- `forwardcollision_ultrasonic_lib`: 장애물 임계값을 정의하는 `ForwardCollisionCalibration`.
- `vs_lane_lib`: 차선 인식 파이프라인과 보조 연산(ROI, 투시 변환, 슬라이딩 윈도우, 칼만 필터).
- `vs_trafficlight_lib`: HSV 임계값, ROI, DBSCAN 파라미터를 가진 신호등 인식 파이프라인.
