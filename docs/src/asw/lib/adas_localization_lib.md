# adas_localization_lib.rs — 로컬라이제이션 헬퍼 라이브러리

- 경로: `src/asw/lib/adas_localization_lib.rs`
- 계층: ASW / Library

## 목적
지도 JSON 파싱, IMU 기준 좌표 보정, yaw 축 판별, 이동 궤적 적분 등의 복잡한 로직을 러너블에서 분리해 재사용 가능하도록 제공합니다.

## 핵심 구성
- `LocalizationRuntime`: 누적 상태(기준 IMU 위치, 마지막 yaw, 좌표 로그 타임스탬프 등)를 묶어 보관합니다.
- `MapData`: 맵 자산을 로드해 inner/outer 레인 waypoint 를 질의할 수 있는 구조체를 제공합니다.
- `process_imu_sample(...)`: IMU 샘플을 받아 기준점 보정, yaw 추정, 이동 거리 누적을 처리해 `DtoLocalizationState`를 생성합니다.
- 보조 함수들(`wrap_angle`, `select_axis` 등)을 통해 yaw 축 탐색, 헤딩과 IMU 오일러 각 비교, 디버그 로그 타이밍 등을 캡슐화합니다.

## 상호 작용
- `calibration::adas_localization`에서 정의한 맵/출발지 프리셋을 사용합니다.
- 결과는 `RteChannels.localization.state_tx`로 전달되어 상위 ADAS 제어 로직에서 사용할 수 있습니다.

