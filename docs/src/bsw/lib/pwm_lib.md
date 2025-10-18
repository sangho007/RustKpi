# pca9685_lib.rs — PCA9685 유틸리티

- 경로: `src/bsw/lib/pwm_lib.rs`
- 계층: BSW / 라이브러리

## 목적
PCA9685 기반 서보/모터 제어에 필요한 상수와 헬퍼 함수를 제공합니다.

## 상수
- I2C 버스: `I2C_BUS = "/dev/i2c-1"`, 주소: `PCA9685_ADDRESS = 0x5f`
- 서보 채널: `SERVO_CHANNELS = [C0, C1, C2]`
- DC 모터 채널: `M1: C15/C14`, `M2: C12/C13`
- 범위: `SERVO_MIN..MAX(205..410)`, `DC_MIN..MAX(0..4096)`

## 열거형
- `Motor { M1, M2 }`
- `Direction { Stop, Forward, Backward }`

## 주요 함수
- `angle_to_pwm(angle: u32) -> u16`: 0~180도 → PWM 값 선형 변환(클램프 포함)
- `percent_to_pwm(percent: u32) -> u16`: 0~100% → PWM 범위 매핑
- `motor_control(pwm, motor, direction, speed)`: 채널 ON=0, OFF=속도 설정
- `motor_stop(pwm, motor)`: 해당 모터 채널 OFF=0

