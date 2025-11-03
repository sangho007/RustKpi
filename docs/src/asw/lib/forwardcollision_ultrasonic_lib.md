# forwardcollision_ultrasonic_lib.rs — 장애물 임계값 캘리브레이션

- 경로: `src/asw/lib/forwardcollision_ultrasonic_lib.rs`
- 계층: ASW / 라이브러리

## 목적
- 전방 장애물 감지 태스크에서 사용할 정지·차선 변경 임계값을 하나의 헬퍼로 노출해 로직과 캘리브레이션을 분리합니다.

## 제공 기능
- `forward_collision_calibration()`은 `ForwardCollisionCalibration::default()`를 반환해 정지 요청 거리(20cm)와 차선 변경 요청 거리(35cm)를 제공하며, 호출자는 구조체를 복사해 바로 사용할 수 있습니다.
- `ForwardCollisionCalibration` 구조체는 필요 시 직접 인스턴스화하거나 튜닝한 값을 넣어 맞춤형 임계값을 정의할 수 있습니다.

## 연관 모듈
- `asw::forwardcollision_ultrasonic`: 반환된 캘리브레이션으로 정지/차선 변경 플래그를 계산합니다.
- `calibration::forward_collision`: 기본 임계값 정의와 튜닝 가이드를 제공합니다.
