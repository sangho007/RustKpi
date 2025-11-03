# bsw/mod.rs — BSW 모듈 인덱스

- 경로: `src/bsw/mod.rs`
- 계층: BSW / 상위 모듈

## 목적
- Basic Software 계층에서 센서·액추에이터 추상화 태스크와 공용 라이브러리를 집약해 상위 애플리케이션이 명시적으로 의존성을 가져갈 수 있도록 합니다.
- 하드웨어별 초기화 코드와 통신 스택을 모듈별로 분리해 유지보수를 용이하게 합니다.

## 하위 모듈
- 공개 모듈: `ecu_abs_cam`, `ecu_abs_pwm`, `ecu_abs_ultrasonic`, `ecu_abs_com`, `ecu_abs_imu`
- 비공개 모듈: `lib` (카메라/초음파/IMU 등 공통 유틸리티 모음)
