# rte_dto.rs — VFB DTO 정의

- 경로: `src/rte/rte_dto.rs`
- 계층: RTE / Virtual Functional Bus

## 목적
ASW/BSW 간 교환되는 데이터 객체(Data Transfer Object, DTO)와 이벤트 열거형을 정의합니다. OpenCV `Mat`은 `Arc<Mat>`으로 감싸 비용을 최소화하면서 멀티스레드 간 안전한 공유가 가능하도록 합니다.

## 주요 타입
- `VfbEvent` 이벤트:
  - `CamRawEvent`, `CamProcessedEvent`, `CamLaneAngleEvent`, `CamBirdEyeViewEvent`, `CamTrafficLightEvent`
  - `UltraSonicRawEvent`, `UltraSonicObstacleDetectedEvent`
  - `ServoCtrlEvent`, `DcMotorCtrlEvent`
- 카메라 DTO:
  - `DtoCamRaw { img: Arc<Mat>, width, height, alive_cnt }`
  - `DtoCamProcessed { img: Arc<Mat>, width, height, alive_cnt }`
  - `DtoCamLaneAngle { angle, alive_cnt }` (작은 타입으로 `Clone` 저비용)
  - `DtoCamBirdEyeView { img: Arc<Mat>, width, height, alive_cnt }`
- 초음파 DTO:
  - `DtoUltraSonicRaw { distance, alive_cnt }`
  - `DtoUltraSonicObstacle { detected, alive_cnt }`
- 액추에이션 DTO:
  - `DtoServoCtrl { channel, angle }`
  - `DtoDcMotorCtrl { direction, speed, alive_cnt }`
- 신호등 DTO:
  - `DtoTrafficLight { traffic_light_color, alive_cnt }`

## 설계 포인트
- 대용량 영상은 `Arc<Mat>`으로 공유해 복사 비용을 줄이고, 계산 파이프라인에서 안전하게 팬아웃.
- 스칼라 위주의 DTO는 `Clone` 저비용으로 손쉽게 브로드캐스트.
- AUTOSAR VFB 시맨틱과 유사하게 신호의 타입/의미를 정형화.

