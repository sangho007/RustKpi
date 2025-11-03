# pwm.rs — PCA9685 PWM 캘리브레이션

- 경로: `src/calibration/pwm.rs`
- 계층: Calibration / Actuator

## 목적
- PCA9685 보드 설정을 중앙에서 정의해 액추에이터 러너블이 하드웨어 파라미터를 공유하도록 합니다.
- 서보·모터 채널 매핑과 듀티비 범위를 명시해 안전한 동작 범위를 강제합니다.

## 제공 항목
- `PwmCalibration`: PCA9685 I2C 주소, 서보 채널, 듀티비 범위, 로그 주기 등을 정의합니다.
- `MotorChannelCalibration`: H-브리지 제어용 듀얼 채널 매핑을 제공합니다.
- 기본값은 서보 2채널, DC 모터 2채널 구성을 가정하고, 서보 듀티비(205~410) 및 전체 PWM 범위를 설정합니다.
