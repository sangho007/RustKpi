# ultrasonic_lib.rs — 초음파 캘리브레이션 래퍼

- 경로: `src/bsw/lib/ultrasonic_lib.rs`
- 계층: BSW / 라이브러리

## 목적
- 초음파 센서 제어에 필요한 GPIO/PWM 파라미터를 한 곳에서 관리해 하드웨어 변경 시 캘리브레이션만 수정하면 되도록 합니다.

## 제공 기능
- `ultrasonic_calibration()`은 `UltrasonicCalibration::default()`를 반환하며, 트리거/에코 핀 번호, 샘플 주기(기본 100ms), 로그 주기(1초)를 포함합니다.

## 연관 모듈
- `bsw::ecu_abs_ultrasonic`: 반환된 캘리브레이션으로 센서를 초기화하고 거리 측정 루프를 구성합니다.
- `calibration::ultrasonic`: 기본 파라미터 정의와 조정 가이드를 담습니다.
