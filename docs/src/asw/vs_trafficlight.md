# vs_trafficlight.rs — 신호등 인지 태스크

- 경로: `src/asw/vs_trafficlight.rs`
- 계층: ASW / Vision (Traffic Light)

## 목적
Raw 카메라 프레임에서 HSV 기반 색 영역 추출 + 클러스터 크기 비교로 신호등 상태를 판별하여 게시합니다.

## 동작 요약
- 입력: `CameraChannels.raw_tx`
- 프레임 드레인: 더 최신 데이터가 있으면 버퍼 비워 최신 프레임만 처리
- 간격 처리: `TRAFFIC_DETECTION_INTERVAL = 5`
- 파이프라인: `convert_to_hsv` → `detect_color_from_hsv`(마스크+모폴로지+DBSCAN) → 상태 유지
- 출력: `TrafficLightSender`에 `DtoTrafficLight`

## 동시성/에러
- `spawn_blocking`으로 분리 실행, 채널 지연은 1초 단위로 로그
- Join 에러를 OpenCV `Error`로 포장

