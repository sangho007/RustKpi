# bsw/mod.rs - BSW 모듈 인덱스

- 경로: `src/bsw/mod.rs`
- 공개 모듈: `ecu_abs_cam`, `ecu_abs_pwm`, `ecu_abs_ultrasonic`, `ecu_abs_com`, `ecu_abs_imu`
- 비공개 하위 모듈: `lib`

BSW(Basic Software) 계층에서 센서·액추에이터·통신 ECU 추상화 태스크를 모읍니다. 카메라, 초음파, PCA9685, USB/TCP 게이트웨이, IMU 디코딩 태스크가 공개되며, 공통 유틸리티는 `lib` 모듈 내부에서만 사용됩니다.
