# asw/lib/mod.rs — ASW 라이브러리 인덱스

- 경로: `src/asw/lib/mod.rs`
- 계층: ASW / 라이브러리 집합

## 목적
- ADAS 소프트웨어 계층 전반에서 공유하는 헬퍼와 캘리브레이션 래퍼를 하나의 모듈 트리로 묶어 재사용성을 높입니다.
- 러너블이 직접 복잡한 세부 구현을 다루지 않도록, 공통 연산을 라이브러리 형태로 캡슐화합니다.

## 하위 모듈
- `adas_localization_lib`: 지도 JSON 파싱, IMU 누적, yaw 축 판별, 기준점 보정을 담당합니다.
- `adas_path_lib`: 지도 그래프 로딩, A* 경로 탐색, 로컬 경로 절단/스무딩 등 Path Planning 유틸리티를 제공합니다.
- `forwardcollision_ultrasonic_lib`: 장애물 임계값을 정의하는 `ForwardCollisionCalibration`.
- `vs_lane_lib`: 차선 인식 파이프라인과 보조 연산(ROI, 투시 변환, 슬라이딩 윈도우, 칼만 필터 옵션).
- `vs_trafficlight_lib`: HSV 임계값, ROI, DBSCAN 파라미터를 가진 신호등 인식 파이프라인.
