# rte_dto.rs - VFB DTO 정의

- 경로: `src/rte/rte_dto.rs`
- 계층: RTE / Virtual Functional Bus

## 목적
BSW와 ASW 간에 오가는 데이터 전송 객체(Data Transfer Object)를 정의합니다. 대용량 영상 데이터는 `Arc`로 공유하고, 스칼라/상태 중심 DTO는 가볍게 복사될 수 있도록 설계했습니다. 텔레메트리(USB/TCP) → IMU 파이프라인도 동일한 방식으로 캡슐화됩니다.

## 카메라 DTO
- `DtoCamRaw { buffer: Arc<CameraBuffer>, width, height, stride, bytes_per_pixel, alive_cnt, color_format }`
  - `as_mat_view()`로 stride/포맷을 고려한 OpenCV `Mat` 뷰를 생성, `as_bgr_mat()`으로 BGR 변환을 보장합니다.
- `DtoCamProcessed { img: Arc<Mat>, width, height, alive_cnt }`: 전처리 단계 출력.
- `DtoCamBirdEyeView { img: Arc<Mat>, width, height, alive_cnt }`: 퍼스펙티브 변환/디버그 시각화 결과.
- `DtoCamLaneAngle { angle: f64, alive_cnt }`: 조향각을 담은 경량 DTO.

## 초음파 DTO
- `DtoUltraSonicRaw { distance: f32, alive_cnt }`
- `DtoUltraSonicObstacle { detected: bool, alive_cnt }`

## 제어 DTO
- `DtoServoCtrl { channel: u8, angle: u32 }`
- `DtoDcMotorCtrl { direction: u32, speed: u32, alive_cnt }`

## 텔레메트리 / IMU DTO
- `DtoTcpTelemetry { payload: Arc<Vec<u8>>, message_size, alive_cnt }`: TCP 길이-프레이밍 패킷을 그대로 담아 IMU 파서가 구독합니다.
- `DtoImu`와 하위 구조체:
  - `DtoImuHeader`: `stamp_ns`, `dt_ns`, `seq`, `session_id`, `clock_domain`, `frame_id`, `child_frame_id`
  - `DtoImuStatus`: 추적 상태, 특징점 개수, 상태 플래그
  - `DtoImuPose`: 위치, 쿼터니언, Euler(yaw/roll/pitch), 공분산, 유효 플래그
  - `DtoImuVelocity`, `DtoImuAcceleration`, `DtoImuGyro`: 월드/바디 축 벡터, 공분산, 유효 여부
  - 메인 DTO는 각 서브 구조체를 `Option`으로 보관하고 `alive_cnt`를 포함합니다.

## 신호등 DTO
- `DtoTrafficLight { traffic_light_color: TrafficLightColor, alive_cnt }`

## 연관 라이브러리
- `CameraBuffer`, `ColorFormat`, `BufferRecycler`는 `rte::lib::camera_lib`에서 재사용되어 libcamera/비디오 버퍼를 안전하게 공유합니다.
- `bsw::lib::imu_proto`는 Swift protobuf payload를 `DtoImu*`로 변환할 때 활용됩니다.
