# ecu_abs_pca9685.rs — ECU Abstraction (PCA9685 액추에이터)

- 경로: `src/bsw/ecu_abs_pwm.rs`
- 계층: BSW / ECU Abstraction (Actuator)

## 목적
PCA9685 PWM 보드를 통해 서보 각도/모터 속도·방향 명령을 실행합니다. 제어 채널을 구독하여 하드웨어에 반영합니다.

## 동작 요약
- I2C 장치(`/dev/i2c-1`)와 PCA9685 초기화 → 50Hz 설정(`prescale=121`) → 채널 ON=0 초기화.
- 서보: `ServoCtrlSender` 구독, 채널별 `angle_to_pwm` 적용 후 `set_channel_off`.
- DC: `DcMotorCtrlSender` 구독, 방향/속도에 따라 `motor_control` 또는 `motor_stop` 호출.
- 상태 요약 로그(서보/모터) 주기 출력.

## 의존성
- 내부: `bsw::lib::pca9685_lib::*`
- 외부: `linux_embedded_hal`, `pwm_pca9685`

## 구성/초기 상태
- 주소: `PCA9685_ADDRESS = 0x5f`
- 서보 초기값: C0=90(조향), C1=180(카메라 좌우), C2=170(카메라 상하)

## 데이터 플로우
- 입력: `ControlChannels.servo_tx`, `ControlChannels.dc_motor_tx`

## 에러 처리
- I2C/PCA9685 초기화 실패 시 로그 후 태스크 종료.
- 런타임 설정 실패는 로그 출력.

