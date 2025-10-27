# forwardcollision_ultrasonic.rs - 전방 초음파 장애물 감지

- 경로: `src/asw/forwardcollision_ultrasonic.rs`
- 계층: ASW / Ultrasonic → Obstacle

## 목적
초음파 RAW 거리 데이터를 임계값 기반으로 이진 장애물 이벤트(`DtoUltraSonicObstacle`)로 변환합니다. FCA(Forward Collision Alert) 기능의 첫 단계이며, 러너블 엔트리포인트는 `runnable_forwardcollision_obstacle_detection`입니다.

## 동작 요약
- `forward_collision_calibration()`에서 임계 거리(`threshold_distance`, 기본 30cm)를 읽어옵니다.
- `UltrasonicChannels.raw_tx.subscribe()`로 거리 DTO 스트림을 구독하고, 최신 샘플을 순차 처리합니다.
- `distance < threshold_distance`이면 `detected = true`로 설정하고 `UltrasonicChannels.obstacle_tx`에 퍼블리시합니다.
- alive_cnt는 내부 카운터로 관리하며, 장애물 DTO에 포함해 상위에서 순서를 확인할 수 있습니다.

## 동시성/에러
- 계산은 `tokio::task::spawn_blocking`으로 별도 스레드에서 수행하여 호출 측 async 런타임과 분리합니다.
- `broadcast::RecvError::Lagged`는 경고 로그를 남기고 다음 루프로 진행합니다.
- 채널이 닫히면 루프를 종료하고, Join 실패는 경고 로그로만 처리합니다.
