# rte_main.rs — RTE 채널 초기화

- 경로: `src/rte/rte_main.rs`
- 계층: RTE / 채널 & 스케줄링

## 목적
Tokio Broadcast 채널을 생성하고, 카메라/초음파/제어 그룹으로 묶은 채널 번들을 제공합니다. 각 태스크는 필요 채널에 `subscribe()` 하여 비동기 파이프라인을 구성합니다.

## 채널 타입 별칭
- 카메라: `CamRawSender`, `CamProcessedSender`, `CamBirdEyeSender`, `CamLaneAngleSender`, `TrafficLightSender`
- 초음파: `UltraRawSender`, `UltraObstacleSender`
- 제어: `ServoCtrlSender`, `DcMotorCtrlSender`

## 버퍼 용량(기본)
- RAW/Processed: 6, BirdEye: 4, LaneAngle/TrafficLight: 8
- UltraRaw/Obstacle: 8, Servo/Dc: 16

## 구조체
- `CameraChannels { raw_tx, processed_tx, bird_eye_tx, lane_angle_tx, traffic_light_tx }`
- `UltrasonicChannels { raw_tx, obstacle_tx }`
- `ControlChannels { servo_tx, dc_motor_tx }`
- `RteChannels { camera, ultrasonic, control }`
- `RteSystem { channels }`

## 초기화 흐름
1) 각 채널에 대해 `broadcast::channel(capacity)` 생성
2) 그룹 구조체로 바인딩 후 `RteSystem`으로 반환

## 데이터 플로우 (VFB 관점)
- 카메라: `raw_tx → processed_tx → bird_eye_tx → lane_angle_tx`, `raw_tx → traffic_light_tx`
- 초음파: `raw_tx → obstacle_tx`
- 제어: `control_tx(servo/dc)`를 BSW 액추에이터가 구독

