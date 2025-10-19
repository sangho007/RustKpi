# rte_dto.rs - VFB DTO 정의

- 경로: `src/rte/rte_dto.rs`
- 계층: RTE / Virtual Functional Bus

## 목적
BSW와 ASW 간에 오가는 데이터 전송 객체(Data Transfer Object)를 정의합니다. 대용량 영상 데이터는 `Arc`로 공유하고, 스칼라 중심 DTO는 가볍게 복사될 수 있도록 설계했습니다.

## 카메라 DTO
- `DtoCamRaw`:
  - 필드: `buffer: Arc<CameraBuffer>`, `width`, `height`, `stride`, `bytes_per_pixel`, `alive_cnt`, `color_format`.
  - 메서드: `as_mat_view()`로 stride/포맷을 고려한 OpenCV `Mat` 뷰를 생성, `as_bgr_mat()`으로 BGR 변환을 보장.
- `DtoCamProcessed`, `DtoCamBirdEyeView`:
  - `img: Arc<Mat>`과 해상도, `alive_cnt`를 보관합니다.
- `DtoCamLaneAngle`:
  - 조향각(f64)과 alive 카운터를 담으며 `Clone`이 저비용입니다.

## 초음파 DTO
- `DtoUltraSonicRaw { distance: f32, alive_cnt }`
- `DtoUltraSonicObstacle { detected: bool, alive_cnt }`

## 제어 DTO
- `DtoServoCtrl { channel: u8, angle: u32 }`
- `DtoDcMotorCtrl { direction: u32, speed: u32, alive_cnt }`

## 신호등 DTO
- `DtoTrafficLight { traffic_light_color: TrafficLightColor, alive_cnt }`

## 이벤트 열거형
- `VfbEvent`는 위 DTO를 모두 감싸 AUTOSAR VFB 스타일의 이벤트 스위치를 제공합니다.

## 연관 라이브러리
- `CameraBuffer`, `ColorFormat`, `BufferRecycler`는 `rte::lib::camera_lib`에서 재사용되어 libcamera/비디오 버퍼를 안전하게 공유합니다.
