# adas_cod.rs — ADAS 제어 러너블

- 경로: `src/asw/adas_cod.rs`
- 계층: ASW / ADAS Control

## 목적
LaneAngle, Ultrasonic, Traffic Light, IMU 텔레메트리를 결합해 서보와 DC 모터 명령을 산출합니다. Classic AUTOSAR 관점에서 Vehicle Dynamics 제어기를 ASW 계층으로 구현한 것입니다.

## 구성 러너블
- `runnable_adas_lateral("ADAS-Lateral", RteChannels)`  
  - 입력: `camera.lane_angle_tx`  
  - 보정: `AdasLateralCalibration` (`lane_to_servo_gain`, `servo_neutral_deg`, `servo_min/max_deg`, `max_servo_delta_deg`, 대상 서보 채널)  
  - 처리: 최신 LaneAngle을 캐시 후 비례 제어 → `max_servo_delta_deg`로 레이트 리밋 → `DtoServoCtrl` 송신  
  - 출력: `control.servo_tx`
- `runnable_adas_longitudinal("ADAS-Longitudinal", RteChannels)`  
  - 입력: `ultrasonic.raw_tx`, `ultrasonic.obstacle_tx`, `camera.traffic_light_tx`  
  - 보정: `AdasLongitudinalCalibration` (`control_period`, 순항 속도, 감속 속도, slowdown/stop 거리, 로그 주기)  
  - 처리: 최신 거리/장애물/신호 상태를 통합해 정지/감속/순항 세 가지 상태 머신을 선택합니다. 장애물 감지, 적색 신호, `stop_distance_cm` 이내 거리는 정지, 황색·소등 신호나 `slowdown_distance_cm` 이내 거리는 감속 속도를 사용하며, 그 외에는 순항 속도를 사용해 `DtoDcMotorCtrl`을 전송합니다.  
  - 출력: `control.dc_motor_tx`

## 주요 특징
- 서보 루프는 `max_servo_delta_deg`를 적용해 루프당 각도 변화량을 제한, 기계적 스트레스를 줄입니다.
- Broadcast 수신은 `try_recv`로 최신 값만 유지해 지연을 줄입니다.
- DC 모터 명령은 값이 변경될 때만 송신하며, `alive_cnt`를 함께 증가시켜 BSW 액추에이터에서 watchdog 용도로 활용할 수 있습니다.
- `AdasLongitudinalCalibration.log_interval`마다 거리·장애물·신호 색상과 선택된 속도를 로그로 남깁니다.
- 신호등이 황색(`TrafficLightColor::Yellow`)이거나 소등(`Off`) 상태면 감속, 적색이면 즉시 정지 상태로 전환합니다.
- IMU 텔레메트리는 현재 정지 판단에 직접 사용하지 않지만, 향후 yaw/가속도 기반 융합을 고려해 `RteChannels.imu` 구독을 유지합니다.

## 연관 캘리브레이션
- `calibration::adas_cod::AdasLateralCalibration`
- `calibration::adas_cod::AdasLongitudinalCalibration`

## 향후 확장
- `src/asw/adas_localization.rs`, `src/asw/adas_path_local.rs`, `src/asw/adas_path_global.rs`는 지도/전역 경로·로컬 궤적 계획용 플레이스홀더로 남아있으며, 추후 Pure Pursuit / Stanley 제어기를 추가할 예정입니다.
