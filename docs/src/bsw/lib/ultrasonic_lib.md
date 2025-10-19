# ultrasonic_lib.rs - 초음파 캘리브레이션 래퍼

- 경로: `src/bsw/lib/ultrasonic_lib.rs`
- 계층: BSW / 라이브러리

`ultrasonic_calibration()` 함수 하나로 래핑되어 있으며 `UltrasonicCalibration::default()`를 그대로 반환합니다. 트리거/에코 핀, 샘플 주기(기본 100ms), 로그 주기(기본 1초)가 포함되며 `bsw::ecu_abs_ultrasonic`에서 센서를 초기화하고 측정 루프를 구성할 때 사용됩니다.
