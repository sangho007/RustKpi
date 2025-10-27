# vs_trafficlight.rs - 신호등 인지 태스크

- 경로: `src/asw/vs_trafficlight.rs`
- 계층: ASW / Vision (Traffic Light)

## 목적
Raw 카메라 프레임을 HSV 기반 컬러 세그멘테이션과 DBSCAN 클러스터링으로 분석해 신호등 색 상태(`DtoTrafficLight`)를 브로드캐스트합니다. 러너블 엔트리포인트는 `runnable_vs_detect_trafficlight`입니다.

## 처리 흐름
1. `TrafficLightCalibration::default()`에서 ROI, HSV 임계값, DBSCAN 파라미터, `detection_interval`(기본 5)을 읽어옵니다. ROI는 카메라 해상도에 맞춰 스케일됩니다.
2. `CameraChannels.raw_tx`를 구독하고, 지연을 줄이기 위해 새 프레임이 있으면 `try_recv`로 버퍼를 비웁니다.
3. 지정된 간격(`alive_cnt % detection_interval == 0`)이거나 현재 상태가 `Off`일 때만 HSV 변환과 색 판정을 수행합니다.
4. `Pipeline::detect_color_from_hsv`는 색별 마스크 생성 → 모폴로지 → DBSCAN → 최대 클러스터 픽셀 수를 비교해 최종 색을 선택합니다.
5. 결과는 `TrafficLightSender`(broadcast)로 전송되며, 마지막 판정은 다음 루프에서 재사용됩니다.

## 동시성/에러
- 연산은 `tokio::task::spawn_blocking`으로 별도 스레드에서 수행합니다.
- 채널 `Lagged` 시 최근 로그 시점(1초 기준) 이후라면 경고를 출력하고 최신 프레임으로 계속 진행합니다.
- Join 실패는 OpenCV `Error`로 감싸 상위 태스크에 보고합니다.
