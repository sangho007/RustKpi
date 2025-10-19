# pwm_lib.rs - PCA9685 유틸리티

- 경로: `src/bsw/lib/pwm_lib.rs`
- 계층: BSW / 라이브러리

## 구성 요소
- `PwmCalibration`: I2C 버스, 디바이스 주소, 서보 채널, 기본 각도, 서보/모터 PWM 범위, 상태 로그 주기를 정의합니다.
- `Motor` / `Direction` 열거형: DC 모터 제어 대상과 회전 방향을 표현합니다.
- 보조 함수:
  - `angle_to_pwm(angle: u32) -> u16`: 0-180도 입력을 서보 PWM off-cycle 값으로 선형 매핑합니다.
  - `percent_to_pwm(percent: u32) -> u16`: 0-100% 속도를 DC 모터 PWM 범위로 변환합니다.
  - `motor_control` / `motor_stop`: 지정된 모터 채널 쌍(IN1/IN2)에 PWM 값을 설정해 방향·속도를 제어합니다.

## 사용 패턴
- 모든 함수는 최신 보정 값을 얻기 위해 내부적으로 `PwmCalibration::default()`를 호출합니다. 하드웨어 구성이 바뀌면 캘리브레이션만 수정하면 됩니다.
- `bsw::ecu_abs_pwm::ea_pca9685_actuator`가 서보 각도와 속도 명령을 실제 PCA9685 레지스터에 반영할 때 이 유틸리티를 사용합니다.
