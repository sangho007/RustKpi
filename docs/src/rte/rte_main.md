# rte_main.rs - RTE 채널 초기화

- 경로: `src/rte/rte_main.rs`
- 계층: RTE / 채널 & 스케줄링

## 목적
Tokio broadcast 채널을 생성하고 카메라, 초음파, 제어 그룹으로 묶은 구조체를 제공합니다. 애플리케이션 전역에서 동일한 채널 핸들을 공유할 수 있도록 `RteSystem`으로 래핑합니다.

## 채널 타입 별칭
- 카메라: `CamRawSender`, `CamProcessedSender`, `CamBirdEyeSender`, `CamLaneAngleSender`, `TrafficLightSender`
- 초음파: `UltraRawSender`, `UltraObstacleSender`
- 제어: `ServoCtrlSender`, `DcMotorCtrlSender`

## 버퍼 용량
- `CAM_RAW_CAPACITY = 2`
- `CAM_PROCESSED_CAPACITY = 6`
- `CAM_BIRD_EYE_CAPACITY = 4`
- `CAM_LANE_ANGLE_CAPACITY = 8`
- `TRAFFIC_LIGHT_CAPACITY = 8`
- `ULTRA_RAW_CAPACITY = 8`
- `ULTRA_OBSTACLE_CAPACITY = 8`
- `SERVO_CTRL_CAPACITY = 16`
- `DC_CTRL_CAPACITY = 16`

Raw 채널은 지연을 최소화하기 위해 깊이를 2로 제한하고, 이후 파이프라인 단계는 더 큰 버퍼로 팬아웃을 허용합니다.

## 구조체 구성
- `CameraChannels { raw_tx, processed_tx, bird_eye_tx, lane_angle_tx, traffic_light_tx }`
- `UltrasonicChannels { raw_tx, obstacle_tx }`
- `ControlChannels { servo_tx, dc_motor_tx }`
- `RteChannels { camera, ultrasonic, control }`
- `RteSystem { channels }`

## 초기화 흐름
1. 각 채널에 대해 `broadcast::channel(capacity)` 호출로 송신자/수신자 쌍을 생성합니다. 초기 수신자는 버린 채널이므로 `_`로 폐기합니다.
2. 송신자 묶음을 구조체에 담아 `RteChannels`를 만들고, 이를 `RteSystem`에 래핑해 반환합니다.

## 데이터 플로우
- 카메라: `raw_tx → processed_tx → bird_eye_tx / lane_angle_tx`, `raw_tx → traffic_light_tx`
- 초음파: `raw_tx → obstacle_tx`
- 제어: 애플리케이션(ASW)에서 `servo_tx`, `dc_motor_tx`로 명령을 게시하면 BSW 액추에이터가 구독해 하드웨어에 반영합니다.
