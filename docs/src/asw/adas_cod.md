# adas_cod.rs — ADAS 제어 러너블

- 경로: `src/asw/adas_cod.rs`
- 계층: ASW / ADAS Control

## 목적
LaneAngle, Ultrasonic, Traffic Light, IMU 텔레메트리를 결합해 서보와 DC 모터 명령을 산출합니다. Classic AUTOSAR 관점에서 Vehicle Dynamics 제어기를 ASW 계층으로 구현한 것입니다.

## 구성 러너블
- `runnable_adas_lateral("ADAS-Lateral", RteChannels)`  
  - 입력: `localization.state_tx`, `path.smoothed_tx`  
  - 보정: `AdasLateralCalibration` (횡오차 PID 게인, 샘플 인덱스, 서보 중립/범위, `max_servo_delta_deg`, 대상 서보 채널)  
  - 처리: 차량 현재 위치와 yaw를 기준으로 직선 참조 축을 만들고, 스무딩 궤적의 지정 샘플(기본 8번째)에서 횡방향 오차를 계산해 PID로 서보 목표각을 생성합니다. 차선 변경 중에는 PID를 리셋하여 급격한 변화를 방지하며, `max_servo_delta_deg` 레이트 리밋을 적용한 뒤 `DtoServoCtrl`을 송신합니다.  
  - 출력: `control.servo_tx`
- `runnable_adas_longitudinal("ADAS-Longitudinal", RteChannels)`  
  - 입력: `ultrasonic.raw_tx`, `ultrasonic.obstacle_tx`, `camera.traffic_light_tx`, `camera.traffic_light_directive_tx`, `path.smoothed_tx`  
  - 보정: `AdasLongitudinalCalibration` (`control_period`, 순항/감속 속도, slowdown/stop 거리, 곡률 감속 임계값, stop hold 시간, 속도 레이트 리밋 등)  
  - 처리: 최신 거리/장애물/신호 상태를 통합하고, 무거운 DTO(`DtoUltraSonicObstacle`, `DtoTrafficLightDirective`, `DtoAdasSmoothedPath`)는 `Arc`로 캐시합니다. 스무딩 경로가 준비되지 않았으면 `no_path` 사유로 정지를 유지하고, 정지 조건이 일정 시간 지속되면 완전 정지, 황색·소등 신호·lane change 요청·가까운 장애물·고 곡률에서는 감속, 그 외에는 순항 속도를 목표로 하며 가감속은 `max_accel_delta_percent` / `max_decel_delta_percent`로 제한합니다.  
  - 출력: `control.dc_motor_tx`

## 주요 특징
- 서보 루프는 `max_servo_delta_deg`를 적용해 루프당 각도 변화량을 제한, 기계적 스트레스를 줄입니다.
- 차량 yaw 직선 참조 대비 스무딩 궤적의 횡오차를 PID로 추종하며, 차선 변경 시에는 PID를 리셋해 안정적으로 전환합니다.
- Broadcast 수신은 `try_recv`로 최신 값만 유지하며, 스무딩 경로·장애물·신호 지시 DTO는 `Arc` 복사만으로 재사용해 지연과 복사 비용을 동시에 줄입니다.
- DC 모터 명령은 값이 변경될 때만 송신하며, `alive_cnt`를 함께 증가시켜 BSW 액추에이터에서 watchdog 용도로 활용할 수 있습니다.
- 종방향 제어는 로그 주기(`log_interval`)마다 거리·장애물·신호 색상·경로 준비 상태와 최종 속도를 출력합니다.
- 신호등이 황색(`TrafficLightColor::Yellow`)이거나 소등(`Off`) 상태면 감속, 적색이면 즉시 정지 상태로 전환하며, 스무딩 경로가 비어 있으면 주행을 시작하지 않습니다.

## 연관 캘리브레이션
- `calibration::adas_cod::AdasLateralCalibration`
- `calibration::adas_cod::AdasLongitudinalCalibration`

## 향후 확장
- `src/asw/adas_localization.rs`, `src/asw/adas_path_local.rs`, `src/asw/adas_path_global.rs`는 지도/전역 경로·로컬 궤적 계획용 플레이스홀더로 남아있으며, 추후 Pure Pursuit / Stanley 제어기를 추가할 예정입니다.
