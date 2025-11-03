# rte_dto.rs — VFB DTO 정의

- 경로: `src/rte/rte_dto.rs`
- 계층: RTE / Virtual Functional Bus

## 목적
BSW와 ASW 간에 오가는 데이터 전송 객체(Data Transfer Object)를 정의합니다. 대용량 영상 데이터는 `Arc`로 공유하고, 스칼라/상태 중심 DTO는 가볍게 복사될 수 있도록 설계했습니다. 텔레메트리(USB/TCP) → IMU 파이프라인도 동일한 방식으로 캡슐화됩니다.

## 카메라 DTO
- `DtoCamRaw { buffer: Arc<CameraBuffer>, width, height, stride, bytes_per_pixel, alive_cnt, color_format }`
  - `as_mat_view()`로 stride/포맷을 고려한 OpenCV `Mat` 뷰를 생성, `as_bgr_mat()`으로 BGR 변환을 보장합니다.
- `DtoCamProcessed { img: Arc<Mat>, width, height, alive_cnt }`: 전처리 단계 출력.
- `DtoCamBirdEyeView { img: Arc<Mat>, width, height, alive_cnt }`: 퍼스펙티브 변환/디버그 시각화 결과.
- `DtoCamLaneAngle { angle: f64, lateral_offset: f64, alive_cnt }`: 조향각과 횡방향 오차를 담은 경량 DTO.

## 초음파 DTO
- `DtoUltraSonicRaw { distance: f32, alive_cnt }`
- `DtoUltraSonicObstacle { stop_requested: bool, lane_change_requested: bool, distance_cm: f32, alive_cnt }`

## 신호등 DTO
- `DtoTrafficLight { traffic_light_color: TrafficLightColor, alive_cnt }`
- `DtoTrafficLightDirective { stop_requested: bool, accelerate_requested: bool, inside_detection_zone: bool, source_color: TrafficLightColor, alive_cnt }`

## 경로 DTO
- `DtoPathWaypoint { lane: LocalizationLane, index: u32, position_xy: [f32; 2], can_change_lane: bool }`: 전역/로컬 경로 waypoint 기본 단위.
- `DtoAdasGlobalPath { map_id, waypoints: Vec<DtoPathWaypoint>, alive_cnt, generated_time_ns }`
- `DtoAdasLocalPath { map_id, origin_alive_cnt, waypoints, alive_cnt, generated_time_ns }`
- `DtoAdasSmoothedPath { map_id, origin_alive_cnt, samples_xy: Vec<[f32; 2]>, alive_cnt, generated_time_ns, lane_change_state: AdasLaneChangeState }`
- `AdasLaneChangeState`: `InnerCruise`, `OuterCruise`, `InnerToOuter`, `OuterToInner`

## 로컬라이제이션 DTO
- `DtoLocalizationState`: 지도 좌표계 위치(`position_map_xy`), 헤딩(`motion_heading_rad`), yaw(`yaw_rad`), IMU 변위 등 누적 상태.
- `DtoLocalizationArrival { arrived, distance_m, timestamp_ns, alive_cnt }`: 목적지 근접 여부를 보고합니다.

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

## 연관 라이브러리
- `CameraBuffer`, `ColorFormat`, `BufferRecycler`는 `rte::lib::camera_lib`에서 재사용되어 libcamera/비디오 버퍼를 안전하게 공유합니다.
- `bsw::lib::imu_proto`는 Swift protobuf payload를 `DtoImu*`로 변환할 때 활용됩니다.
- `asw::lib::adas_path_lib`는 경로 DTO를 구성하고 곡률 계산을 수행합니다.
