# rte_main.rs — RTE 채널 초기화

- 경로: `src/rte/rte_main.rs`
- 계층: RTE / 채널 & 스케줄링

## 목적
Tokio broadcast 채널을 생성하고 카메라, 초음파, 제어, TCP 텔레메트리, IMU 그룹으로 묶은 구조체를 제공합니다. 애플리케이션 전역에서 동일한 채널 핸들을 공유할 수 있도록 `RteSystem`으로 래핑합니다.

## 채널 타입 별칭
- 카메라: `CamRawSender`, `CamProcessedSender`, `CamBirdEyeSender`, `CamLaneAngleSender`, `TrafficLightSender`, `TrafficLightDirectiveSender`
- 초음파: `UltraRawSender`, `UltraObstacleSender`
- 제어: `ServoCtrlSender`, `DcMotorCtrlSender`
- 텔레메트리/IMU: `TcpRawSender`, `ImuParsedSender`
- 로컬라이제이션: `LocalizationStateSender`, `LocalizationArrivalSender`
- 경로: `GlobalPathSender`, `LocalPathSender`, `SmoothedPathSender`

## 버퍼 용량
- `CAM_RAW_CAPACITY = 2`
- `CAM_PROCESSED_CAPACITY = 6`
- `CAM_BIRD_EYE_CAPACITY = 4`
- `CAM_LANE_ANGLE_CAPACITY = 8`
- `TRAFFIC_LIGHT_CAPACITY = 8`
- `TRAFFIC_LIGHT_DIRECTIVE_CAPACITY = 8`
- `ULTRA_RAW_CAPACITY = 8`
- `ULTRA_OBSTACLE_CAPACITY = 8`
- `SERVO_CTRL_CAPACITY = 16`
- `DC_CTRL_CAPACITY = 16`
- `TCP_RAW_CAPACITY = 16`
- `IMU_PARSED_CAPACITY = 16`
- `LOCALIZATION_STATE_CAPACITY = 16`
- `LOCALIZATION_ARRIVAL_CAPACITY = 8`
- `PATH_GLOBAL_CAPACITY = 4`
- `PATH_LOCAL_CAPACITY = 8`
- `PATH_SMOOTHED_CAPACITY = 8`

Raw 채널은 지연을 최소화하기 위해 깊이를 2로 제한하고, 이후 파이프라인 단계는 더 큰 버퍼로 팬아웃을 허용합니다.

## 구조체 구성
- `CameraChannels { raw_tx, processed_tx, bird_eye_tx, lane_angle_tx, traffic_light_tx, traffic_light_directive_tx }`
- `UltrasonicChannels { raw_tx, obstacle_tx }`
- `ControlChannels { servo_tx, dc_motor_tx }`
- `TcpChannels { telemetry_tx }`
- `ImuChannels { raw_tx, parsed_tx }`
- `LocalizationChannels { state_tx, arrival_tx }`
- `PathChannels { global_tx, local_tx, smoothed_tx }`
- `RteChannels { camera, ultrasonic, control, com, imu, localization, path }`
- `RteSystem { channels }`

## 초기화 흐름
1. 각 채널에 대해 `broadcast::channel(capacity)` 호출로 송신자/수신자 쌍을 생성합니다. 초기 수신자는 버린 채널이므로 `_`로 폐기합니다.
2. 송신자 묶음을 구조체에 담아 `RteChannels`를 만들고, 이를 `RteSystem`에 래핑해 반환합니다.

- 경로/로컬라이제이션: `localization.state_tx` → 전역/로컬 경로 → 스무딩 경로 → ADAS 제어/GUI
