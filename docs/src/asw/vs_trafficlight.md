# vs_trafficlight.rs - 신호등 인지 태스크

- 경로: `src/asw/vs_trafficlight.rs`
- 계층: ASW / Vision (Traffic Light)

## 목적
Raw 카메라 프레임을 HSV 기반 컬러 세그멘테이션과 DBSCAN 클러스터링으로 분석해 신호등 색 상태(`DtoTrafficLight`)와 주행 지시(`DtoTrafficLightDirective`)를 브로드캐스트합니다. 러너블 엔트리포인트는 `runnable_vs_detect_trafficlight`이며, ADAS Localization이 알려주는 차량 위치가 사거리 맵의 감지 구간에 들어왔을 때만 신호를 판별합니다.

## 처리 흐름
1. `TrafficLightCalibration::default()`에서 ROI, HSV 임계값, DBSCAN 파라미터, `detection_interval`(기본 5)을 읽어옵니다.
2. `CameraChannels.raw_tx`와 `LocalizationChannels.state_tx`를 동시에 구독해 최신 카메라 프레임과 위치 데이터를 확보합니다.
3. Localization 정보가 `TRAFFIC_LIGHT_DETECTION_ZONES`에 정의된 구간(기본: Crossroad 맵의 `-0.3 < x < 0.3`, `-1.24 < y < -0.8`)에 있을 때만 HSV 변환 및 색 판정을 수행합니다. 영역 밖에서는 색상을 `Off`로 유지합니다.
4. `Pipeline::detect_color_from_hsv`는 색별 마스크 생성 → 모폴로지 → DBSCAN → 최대 클러스터 픽셀 수를 비교해 최종 색을 선택합니다.
5. 결과 색상은 `TrafficLightSender`로, 동시에 빨간불일 때 정지 요청·초록불일 때 가속 요청을 담은 `DtoTrafficLightDirective`를 `TrafficLightDirectiveSender`로 전송합니다.

## 동시성/에러
- 연산은 `tokio::task::spawn_blocking`으로 별도 스레드에서 수행합니다.
- 채널 `Lagged` 시 최근 로그 시점(1초 기준) 이후라면 경고를 출력하고 최신 프레임으로 계속 진행합니다.
- Join 실패는 OpenCV `Error`로 감싸 상위 태스크에 보고합니다.
