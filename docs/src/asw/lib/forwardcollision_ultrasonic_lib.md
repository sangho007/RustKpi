# forwardcollision_ultrasonic_lib.rs - 장애물 임계값 캘리브레이션

- 경로: `src/asw/lib/forwardcollision_ultrasonic_lib.rs`
- 계층: ASW / 라이브러리

`forward_collision_calibration()` 함수가 `ForwardCollisionCalibration::default()`를 반환합니다. 현재 설정은 `threshold_distance = 30.0f32`로, `asw::forwardcollision_ultrasonic` 태스크에서 장애물 판정 기준으로 사용됩니다. 거리 임계값을 조정할 때는 캘리브레이션만 수정하면 나머지 로직은 변경할 필요가 없습니다.
