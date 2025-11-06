//! PCA9685에 대한 공통 제어 유틸리티.
//! - 서보/모터 채널 맵핑과 보간식을 한곳에서 관리해 ECU 모듈에서 쉽게 재사용한다.
//! - 캘리브레이션 값은 `PwmCalibration`에서 읽어와 일관된 하드웨어 동작을 보장한다.

use crate::calibration::pwm::{MotorChannelCalibration, PwmCalibration};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Channel, Pca9685};

/// 제어 가능한 모터 채널을 구분하는 열거형.
/// - `M1`, `M2`는 차량에 장착된 두 개의 구동 모터에 대응한다.
pub enum Motor {
    M1,
    M2,
}

/// 모터 회전 방향을 나타내는 열거형.
/// - `Stop`은 PWM 출력을 0으로 만들어 모터를 정지시킨다.
/// - `Forward`, `Backward`는 H-브리지를 통해 회전 방향을 결정한다.
pub enum Direction {
    Stop,
    Forward,
    Backward,
}

/// 전역 캘리브레이션 설정을 편의상 재사용한다.
fn calibration() -> PwmCalibration {
    PwmCalibration::default()
}

/// I2C 버스 식별자를 반환한다(예: `/dev/i2c-1`).
pub fn i2c_bus() -> &'static str {
    calibration().i2c_bus
}

/// PCA9685 장치의 I2C 주소를 반환한다.
pub fn device_address() -> u8 {
    calibration().device_address
}

/// 서보 모터 채널 매핑 배열을 반환한다.
pub fn servo_channels() -> [Channel; 2] {
    calibration().servo_channels
}

/// 원하는 각도(0-180도)를 PCA9685의 PWM off-cycle 값으로 변환한다.
/// - 서보 모터의 캘리브레이션 범위(`servo_min`, `servo_max`)를 선형 보간한다.
/// - 입력 값은 0~180도 범위로 강제(clamp)해 안전한 동작을 보장한다.
pub fn angle_to_pwm_steer(angle: u32) -> u16 {
    // 안전을 위해 각도를 0-180도 범위로 제한
    let clamped_angle = angle.clamp(0, 180);
    let converted_angle = 180 - clamped_angle;
    let calib = calibration();
    // SERVO_MIN과 SERVO_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = calib.servo_min as f64
        + (converted_angle as f64 / 180.0) * (calib.servo_max - calib.servo_min) as f64;
    pwm_value.round() as u16
}
pub fn angle_to_pwm_ultrasonic(angle: u32) -> u16 {
    // 안전을 위해 각도를 0-180도 범위로 제한
    let clamped_angle = angle.clamp(0, 180);
    let calib = calibration();
    // SERVO_MIN과 SERVO_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = calib.servo_min as f64
        + (clamped_angle as f64 / 180.0) * (calib.servo_max - calib.servo_min) as f64;
    pwm_value.round() as u16
}

/// DC 모터를 제어한다.
/// - `speed`는 0~4095 범위로 제어되며, 내부에서 클램프 처리한다.
/// - 전/후진 시 서로 다른 브리지 핀을 구동하도록 듀티비를 설정한다.
pub fn motor_control(pwm: &mut Pca9685<I2cdev>, motor: Motor, direction: Direction, speed: u16) {
    let calib = calibration();
    let (in1, in2) = match motor {
        Motor::M1 => motor_channels(&calib.motor_m1),
        Motor::M2 => motor_channels(&calib.motor_m2),
    };

    let (in1_speed, in2_speed) = match direction {
        Direction::Forward => (speed.clamp(0, 4095), 0),
        Direction::Backward => (0, speed.clamp(0, 4095)),
        Direction::Stop => (0, 0),
    };

    // PWM 신호의 시작점(ON)을 0으로 설정합니다.
    let _ = pwm.set_channel_on(in1, 0);
    let _ = pwm.set_channel_on(in2, 0);
    // PWM 신호의 종료점(OFF)을 설정하여 속도를 제어합니다.
    let _ = pwm.set_channel_off(in1, in1_speed);
    let _ = pwm.set_channel_off(in2, in2_speed);
}

/// 특정 모터를 정지시킨다.
/// - 두 제어 핀 모두 OFF 사이클을 0으로 만들어 전류를 차단한다.
pub fn motor_stop(pwm: &mut Pca9685<I2cdev>, motor: Motor) {
    let calib = calibration();
    let (in1, in2) = match motor {
        Motor::M1 => motor_channels(&calib.motor_m1),
        Motor::M2 => motor_channels(&calib.motor_m2),
    };

    let _ = pwm.set_channel_off(in1, 0);
    let _ = pwm.set_channel_off(in2, 0);
}

/// 0~100% 속도 명령을 PWM off-cycle 값으로 변환한다.
/// - DC 모터 채널의 최소/최대 듀티비(`dc_min`, `dc_max`)를 기반으로 선형 보간한다.
/// - 계산된 값이 허용 범위를 벗어나면 다시 클램프한다.
pub fn percent_to_pwm(percent: u32) -> u16 {
    // 안전을 위해 입력 퍼센트를 0-100으로 제한
    let clamped_percent = percent.clamp(0, 100);
    let calib = calibration();
    // DC_MIN과 DC_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = calib.dc_min as f64
        + (clamped_percent as f64 / 100.0) * (calib.dc_max - calib.dc_min) as f64;
    // 계산된 값이 DC 범위를 벗어나지 않도록 한 번 더 클램프
    let pwm_value = pwm_value.round();
    pwm_value.clamp(calib.dc_min as f64, calib.dc_max as f64) as u16
}

/// 캘리브레이션 정보에서 IN1/IN2 채널 쌍을 추출한다.
fn motor_channels(calib: &MotorChannelCalibration) -> (Channel, Channel) {
    (calib.in1, calib.in2)
}
