# vs_lane.rs — 차선 인식 태스크

- 경로: `src/asw/vs_lane.rs`
- 계층: ASW / Vision (Lane)

## 목적
- 카메라 원시 스트림에서 차선을 검출하고 조향각 DTO를 생성해 후속 제어 모듈이 사용할 수 있도록 합니다.
- 전처리와 각도 계산을 분리해 파이프라인 단계를 독립적으로 조정할 수 있게 합니다.

## 구성 러너블
1. `runnable_vs_preprocessing`  
   - 입력: `CameraChannels.raw_tx`  
   - 처리: `LaneCalibration`의 필터/모폴로지 설정을 이용해 Gray → Blur → Canny → Closing을 수행하고 `DtoCamProcessed`로 게시합니다.  
   - 캐싱: `LaneRuntimeCalibration.process_interval`(0이면 매 프레임)을 기준으로 전처리를 건너뛰고 마지막 결과(Arc<Mat>)를 재사용할 수 있습니다.
2. `runnable_vs_get_lane_angle`  
   - 입력: `CameraChannels.processed_tx`  
   - 처리: ROI, 투시 변환, 슬라이딩 윈도우 추적으로 버드아이 뷰와 조향각을 계산합니다(칼만 필터는 사용하지 않습니다).  
   - 출력: `DtoCamBirdEyeView`, `DtoCamLaneAngle`

두 단계 모두 `LaneCalibration::preset(LaneCalibrationPreset::Vga640x480)`을 사용해 ROI, 투시 행렬, 슬라이딩 윈도우 파라미터 등을 가져옵니다. 현재 구현은 칼만 필터 없이 최신 프레임 계산값을 그대로 사용합니다.

## 동시성 및 타이밍
- 각 Runnable은 `std::thread::Builder::spawn`으로 전용 스레드를 생성하고, 메인 async 태스크는 스레드 종료를 `oneshot`으로 기다립니다.
- Broadcast 백오프: `blocking_recv` 후 `try_recv`로 최신 프레임까지 드레인하여 지연을 최소화합니다.
- 처리 FPS는 매 초 `Instant` 측정으로 로그에 출력합니다.

## 장애 대응
- 채널 `Lagged` 이벤트는 경고만 출력하고 최신 데이터를 계속 처리합니다.
- 채널 `Closed` 발생 시 루프를 종료하며, 스레드 Join 결과를 OpenCV `Error`로 매핑해 상위에 전달합니다.
