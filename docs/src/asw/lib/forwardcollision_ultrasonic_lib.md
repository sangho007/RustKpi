# forwardcollision_ultrasonic_lib.rs - 장애물 임계값 캘리브레이션

- 경로: `src/asw/lib/forwardcollision_ultrasonic_lib.rs`
- 계층: ASW / 라이브러리

`forward_collision_calibration()` 함수가 `ForwardCollisionCalibration::default()`를 반환합니다. 현재 설정은 `stop_request_distance_cm = 20.0`, `lane_change_request_distance_cm = 35.0`이며, `asw::forwardcollision_ultrasonic` 태스크에서 정지/차선 변경 판정 기준으로 사용됩니다. 임계값을 조정할 때는 캘리브레이션만 수정하면 나머지 로직은 변경할 필요가 없습니다.
