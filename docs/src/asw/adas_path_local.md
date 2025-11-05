# adas_path_local.rs — 로컬 경로 생성 및 스무딩 러너블

- 경로: `src/asw/adas_path_local.rs`
- 계층: ASW / ADAS Path Planning

## 목적
전역 경로와 로컬라이제이션 결과를 기반으로 차량 주변 구간을 잘라내고, 5차 다항식 스무딩을 통해 제어 입력으로 사용 가능한 단기 궤적을 생성합니다. 생성된 로컬 경로(`DtoAdasLocalPath`)와 스무딩 경로(`DtoAdasSmoothedPath`)는 ADAS 제어와 GUI 시각화에서 활용합니다.

## 구성 러너블
### `runnable_adas_path_local("ADAS-Path-Local", RteChannels)`
- **입력**: `path.global_tx`(전역 경로), `localization.state_tx`(로컬라이제이션 상태)
- **보정**: `AdasPathLocalCalibration::default()`에서 `waypoint_window`(기본 10) 값을 읽어와 주변 waypoint 개수를 결정합니다.
- **동작**
  1. 전역 경로와 로컬라이제이션 상태를 각각 최신 `Arc`로 캐시해 DTO 복사를 피합니다.
  2. 두 데이터가 모두 준비되면 `try_publish_local_path`를 호출해 현재 위치와 가장 가까운 waypoint부터 `waypoint_window` 개수만큼 잘라 `DtoAdasLocalPath`로 브로드캐스트합니다.
  3. 1초 주기로 전역 경로 alive 카운트, 절편 구간, waypoint 개수를 로그로 남깁니다.
- **출력**: `path.local_tx`

### `runnable_adas_path_smoothing("ADAS-Path-Smooth", RteChannels)`
- **입력**: `path.local_tx`(로컬 경로), `localization.state_tx`
- **보정**: `AdasPathLocalCalibration::default()`의 `smoothing_sample_count`(기본 20)과 `smoothing_skip_head`(기본 3)을 사용합니다.
- **동작**
  1. 최신 로컬 경로와 로컬라이제이션 상태(`Arc`)가 모두 도착하면 `smooth_local_path`를 호출해 프레네 좌표 기반 5차 다항식으로 궤적을 스무딩합니다. 스무딩 시 선행 `smoothing_skip_head` 개 waypoint는 제외해 차량 직후 노이즈를 줄입니다.
  2. 스무딩이 실패할 경우 원본 waypoint 좌표를 그대로 사용하고 경고를 출력합니다.
  3. 차선 구성에 따라 `AdasLaneChangeState`를 판정하여 DTO에 포함합니다.
  4. 1초마다 스무딩된 샘플 수, 차선 상태를 로그로 남깁니다.
- **출력**: `path.smoothed_tx`

## 발행 DTO
- `DtoAdasLocalPath { map_id, origin_alive_cnt, waypoints, alive_cnt, generated_time_ns }`
- `DtoAdasSmoothedPath { map_id, origin_alive_cnt, samples_xy, alive_cnt, generated_time_ns, lane_change_state }`

## 예외 처리
- 채널이 `Lagged`되면 누락 개수를 경고로 출력하고 최신 메시지 처리만 계속합니다.
- 채널이 `Closed`되면 해당 러너블을 종료하면서 원인을 로그로 남깁니다.

## 연관 모듈
- `asw::lib::adas_path_lib`: local path 절단/스무딩 유틸(`try_publish_local_path`, `smooth_local_path`, `determine_lane_change_state` 등)을 제공합니다.
- `calibration::adas_path::AdasPathLocalCalibration`: waypoint 윈도우, 스무딩 샘플 수, 선행 제외 개수를 정의합니다.
- `asw::adas_path_global`: 최신 전역 경로를 공급합니다.
- `main_runtime`: `DtoAdasLocalPath`와 `DtoAdasSmoothedPath`를 Path View에서 시각화합니다.
