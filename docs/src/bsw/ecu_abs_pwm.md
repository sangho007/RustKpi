# ecu_abs_pwm.rs — ECU Abstraction (PCA9685 Actuator)

- 경로: `src/bsw/ecu_abs_pwm.rs`
- 계층: BSW / ECU Abstraction (Actuator)

## 목적
PCA9685 기반 PWM 보드를 통해 서보 각도와 DC 모터 속도를 하드웨어에 반영합니다. `ControlChannels`의 broadcast 명령을 구독해 실시간으로 I2C 명령을 전송합니다.

## 초기화 순서
1. `PwmCalibration::default()`에서 I2C 버스 경로, 디바이스 주소, 서보 채널, 기본 각도, 로그 주기를 읽어옵니다.
2. `linux_embedded_hal::I2cdev`로 `/dev/i2c-1`(기본) 버스를 열고 `pwm_pca9685::Pca9685`를 생성합니다.
3. Prescale 121(약 50 Hz)과 `enable()` 호출로 PWM을 활성화하고, 모든 서보 채널의 ON 타이밍을 0으로 초기화합니다.
4. 보정 각도(`servo_default_angles`)를 `angle_to_pwm` 변환 후 채널 OFF 값으로 세팅합니다.

## 명령 처리 루프
- `servo_tx.subscribe()`와 `dc_motor_tx.subscribe()` 두 broadcast 수신자를 `tokio::select!`로 동시에 대기합니다.
- 서보 명령(`DtoServoCtrl`):
  - 채널 인덱스를 보정 배열에 매핑하고 `angle_to_pwm`으로 오프셋을 계산합니다.
  - 변경이 감지되거나 `servo_log_interval`이 지났을 때 통합 상태 로그를 출력합니다.
- 모터 명령(`DtoDcMotorCtrl`):
  - `Direction`(정방향=1, 역방향=2, 정지=0)에 따라 `motor_control` 또는 `motor_stop`을 호출합니다.
  - 최근 상태가 바뀌었거나 `dc_log_interval`이 경과하면 요약 로그를 남깁니다.

## 의존 모듈
- `bsw::lib::pwm_lib`:
  - 캘리브레이션 래퍼(`PwmCalibration`), 유틸 함수(`angle_to_pwm`, `percent_to_pwm`), 모터 방향 유틸.
- 외부 크레이트: `linux_embedded_hal`, `pwm_pca9685`

## 장애 대응
- I2C 또는 PCA9685 초기화 실패 시 경고 로그 후 태스크를 종료합니다.
- 채널 `Lagged` 이벤트는 경고만 출력하고 루프를 계속합니다. `Closed`가 발생하면 루프를 중단하고 PCA9685를 정리(`disable`, `destroy`)합니다.

## 데이터 플로우
- 입력: `ControlChannels.servo_tx`, `ControlChannels.dc_motor_tx`
- 출력: 하드웨어 PCA9685 보드에 직접 I2C 명령을 전송(별도 DTO 없음)
