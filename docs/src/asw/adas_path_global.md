# adas_path_global.rs — 전역 경로 계획 러너블

- 경로: `src/asw/adas_path_global.rs`
- 계층: ASW / ADAS Path Planning

## 목적
지도 기반 waypoint 그래프(`asw::lib::adas_path_lib`)를 사용해 현재 차량 위치와 목적지 사이의 전역 경로를 주기적으로 계산하고, 로컬 경로/제어 모듈이 참조할 수 있도록 `DtoAdasGlobalPath`를 브로드캐스트합니다. 장애물 센서와 차선 변경 요청을 반영해 차단 구간을 동적으로 회피합니다.

## 주요 흐름
1. **초기화**
   - `LOCALIZATION_ACTIVE_SCENARIO`에서 목표 목적지를 확인하고, `PathGraph::load`로 지도 JSON을 읽어 inner/outer 레인 그래프를 구성합니다.
   - 목적지 waypoint 유효성을 검사한 뒤, RTE 채널(`localization.state_tx`, `ultrasonic.obstacle_tx`) 구독자를 생성하고 `path.global_tx` 송신자를 복제합니다.
2. **Localization 업데이트**
   - 최신 `DtoLocalizationState`만 유지해 차량 위치를 추적합니다. 샘플이 누락되면 경고만 출력하고 계속 진행합니다.
3. **장애물/차선 변경 처리**
   - `DtoUltraSonicObstacle`을 수신하면 차선 변경 요청 여부와 거리 기반 차단 범위를 계산해, 목표 waypoint 앞 구간을 `blocked_nodes`에 추가합니다.
   - 차선 변경 실패 시 재시도를 지연시키기 위해 `lane_change_retry_cooldown`를 사용합니다.
4. **주기적 재계획**
   - `AdasPathGlobalCalibration.replanning_period`마다 현재 위치에서 목적지까지 A* 탐색을 수행합니다.
   - 일반 모드와 강제 차선 변경 모드(`PathPlanningMode::ForceLaneChange`)를 상황에 맞게 선택하고, 성공 시 `publish_global_path`를 호출해 경로를 브로드캐스트합니다.
   - 경로 생성이 실패하면 원인을 로그로 남기고 다음 주기를 기다립니다.

## 발행되는 DTO
- `DtoAdasGlobalPath { map_id, waypoints, alive_cnt, generated_time_ns }`
  - 각 waypoint는 `DtoPathWaypoint { lane, index, position_xy, can_change_lane }`를 포함합니다.
  - `alive_cnt`는 32비트 wrap-around 카운터, `generated_time_ns`는 `now_timestamp_ns()`에서 생성됩니다.

## 장애물 회피 로직
- 초음파 센서가 정지 요청이나 차선 변경 요청을 보낼 경우, 현재 경로에서 차량 위치 이후 waypoint 중 차량 헤딩과 `AdasPathGlobalCalibration::obstacle_block_heading_tolerance_deg`(기본 ±10°) 이내에 있는 지점만 누적 거리가 `distance_cm + obstacle_block_margin_m` 이내일 때 차단합니다.
- 차단된 waypoint는 `obstacle_block_timeout`이 경과하면 자동으로 해제되며, 매 주기마다 정리됩니다.

## 로깅
- 1초 간격으로 경로 업데이트 로그를 출력합니다. 로그에는 계획 모드, 시작/목표 차선, waypoint 개수, 차선 변경 횟수가 포함됩니다.
- 채널 지연(`Lagged`)이나 종료(`Closed`) 상태를 감지하면 경고 또는 종료 메시지를 출력해 운영자가 상태를 파악할 수 있게 합니다.

## 연관 모듈
- `calibration::adas_path::AdasPathGlobalCalibration`: 재계획 주기, 차선 변경 비용, 장애물 차단 거리 등 파라미터 제공.
- `asw::lib::adas_path_lib`: 지도 로딩, A* 탐색, DTO 생성 유틸리티.
- `asw::adas_path_local`: 전역 경로를 잘라 로컬 경로로 변환.
- `asw::adas_path_local::runnable_adas_path_smoothing`: 로컬 경로를 스무딩해 제어 입력으로 사용.
