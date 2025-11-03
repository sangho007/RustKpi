# adas_localization.rs — 지도 기반 로컬라이제이션 러너블

- 경로: `src/asw/adas_localization.rs`
- 계층: ASW / ADAS Localization

## 목적
IMU 텔레메트리를 누적해 차량의 현재 위치와 yaw 를 추정하고(`DtoLocalizationState`), 도착지 근접 여부를 판단하는 `DtoLocalizationArrival`을 별도로 브로드캐스트합니다. 지도 JSON과 시나리오 캘리브레이션을 바탕으로 좌표계를 구성합니다.

## 구성 러너블
### `runnable_adas_localization("ADAS-Localization", RteChannels)`
- **입력**: `imu.parsed_tx`
- **프로시저**
  1. `LOCALIZATION_ACTIVE_SCENARIO`에서 지도와 시작 waypoint 를 읽어와 기준 좌표계를 설정합니다.
  2. IMU 브로드캐스트를 구독하고, `try_recv`로 큐를 비워 최신 샘플만 사용합니다.
  3. `adas_localization_lib::process_imu_sample`을 호출해 기준점 보정, 프레임 변환, yaw 축 판별, 이동 거리 누적을 수행하고 `DtoLocalizationState`를 생성합니다.
  4. 성공 시 `localization.state_tx`로 상태를 브로드캐스트하고, 오류는 경고 로그로 남깁니다.
- **출력**: `DtoLocalizationState` (위치, yaw, 헤딩, alive 카운터 포함)

### `runnable_adas_arrival("ADAS-Arrival", RteChannels)`
- **입력**: `localization.state_tx`
- **프로시저**
  1. 동일한 시나리오 맵에서 목적지 waypoint를 찾고, 임계 거리(`LOCALIZATION_ARRIVAL_THRESHOLD_M`)를 계산합니다.
  2. 최신 Localization 상태만 유지하며, 차량이 임계 거리 안으로 들어오면 도착 이벤트를 `DtoLocalizationArrival`로 브로드캐스트합니다.
  3. 차량이 다시 멀어지면 재무장을 위해 도착 플래그를 해제합니다.
- **출력**: `DtoLocalizationArrival { arrived, distance_m, timestamp_ns, alive_cnt }`

## 오류 처리
- 지도 파일 로딩 실패나 잘못된 waypoint는 즉시 로그를 남기고 러너블 실행을 중단합니다.
- IMU/Localization 채널이 `Lagged`일 때는 누락 개수를 경고만 출력하고 최신 샘플로 복구합니다.
- 채널이 `Closed`되면 원인을 알리고 루프를 종료합니다.

## 연관 모듈
- `asw::lib::adas_localization_lib`: 지도 파싱(`MapData`), yaw 축 결정, IMU 누적 로직을 제공합니다.
- `calibration::adas_localization`: 맵/시작/도착 waypoint, 임계 거리, 시나리오 선택 상수를 정의합니다.
- `main_runtime`: Path View에서 `DtoLocalizationState`를 참조해 현재 위치와 헤딩을 표시합니다.
