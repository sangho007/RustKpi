# uss_forwardcollision.rs — 전방 충돌 감지(FCA)

- 경로: `src/asw/forwardcollision_ultrasonic.rs`
- 계층: ASW / UltraSonic → Obstacle

## 목적
초음파 Raw 거리 데이터를 임계값 기반으로 장애물 감지 이벤트(`DtoUltraSonicObstacle`)로 변환합니다.

## 동작 요약
- 입력 구독: `UltrasonicChannels.raw_tx`
- 판정: `distance < THRESHOLD_DISTANCE(기본 30cm)` → `detected = true`
- 출력: `UltrasonicChannels.obstacle_tx`

## 의존성
- 내부: `asw::lib::uss_lib::THRESHOLD_DISTANCE`

## 동시성/에러
- `spawn_blocking`으로 CPU 부담을 분리.
- 채널 `Lagged/Closed` 로그 처리.

