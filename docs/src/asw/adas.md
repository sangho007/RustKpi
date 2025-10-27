# adas_cod.rs — ADAS 제어 러너블

- 경로: `src/asw/adas_cod.rs`
- 계층: ASW / ADAS Control

## 목적
LaneAngle, Ultrasonic, Traffic Light, IMU 텔레메트리를 결합해 서보와 DC 모터 명령을 산출합니다. Classic AUTOSAR 관점에서 Vehicle Dynamics 제어기를 ASW 계층으로 구현한 것입니다.

## 구성 러너블
- `runnable_adas_lateral("ADAS-Lateral", RteChannels)`  
  - 입력: `camera.lane_angle_tx`  
  - 보정: `AdasLateralCalibration` (`lane_to_servo_gain`, `servo_neutral_deg`, 각도 범위, 레이트 리밋, 대상 서보 채널)  
  - 처리: 최신 LaneAngle을 캐시 후 비례 제어 → 레이트 리밋 → `DtoServoCtrl` 송신  
  - 출력: `control.servo_tx`
- `runnable_adas_longitudinal("ADAS-Longitudinal", RteChannels)`  
  - 입력: `ultrasonic.raw_tx`, `ultrasonic.obstacle_tx`, `camera.traffic_light_tx`  
  - 보정: `AdasLongitudinalCalibration` (`control_period`, 순항 속도, 감속 속도, slowdown/stop 거리, 로그 주기)  
  - 처리: 최신 거리/장애물/신호 상태를 통합해 정지/감속/순항 세 가지 상태 머신을 선택 후 `DtoDcMotorCtrl` 전송  
  - 출력: `control.dc_motor_tx`

## 주요 특징
- Broadcast 수신은 `try_recv`로 최신 값만 유지해 지연을 줄입니다.
- `alive_cnt`는 모터 명령이 전달될 때마다 증가해 BSW에서 watchdog 용도로 활용할 수 있습니다.
- 1초 주기로 상태 로그를 출력해 현재 판단 근거(거리, 장애물, 신호 색상)를 추적합니다.
- IMU 텔레메트리는 현재 정지 판단에 직접 사용하지 않지만, 향후 yaw/가속도 기반 융합을 고려해 `RteChannels.imu` 구독을 유지합니다.

## 연관 캘리브레이션
- `calibration::adas_cod::AdasLateralCalibration`
- `calibration::adas_cod::AdasLongitudinalCalibration`

## 향후 확장
- `src/asw/adas_localization.rs`, `src/asw/adas_trajectory.rs`는 지도/경로 계획용 플레이스홀더로 남아있으며, 추후 Pure Pursuit / Stanley 제어기를 추가할 예정입니다.
