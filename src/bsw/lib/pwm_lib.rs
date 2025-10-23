use crate::calibration::pwm::{MotorChannelCalibration, PwmCalibration};
use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Channel, Pca9685};

// 제어할 모터를 나타내는 열거형
pub enum Motor {
    M1,
    M2,
}

// 모터 회전 방향을 나타내는 열거형
pub enum Direction {
    Stop,
    Forward,
    Backward,
}

fn calibration() -> PwmCalibration {
    PwmCalibration::default()
}

pub fn i2c_bus() -> &'static str {
    calibration().i2c_bus
}

pub fn device_address() -> u8 {
    calibration().device_address
}

pub fn servo_channels() -> [Channel; 2] {
    calibration().servo_channels
}

/// 원하는 각도(0-180)를 PCA9685의 PWM off-cycle 값으로 변환합니다.
pub fn angle_to_pwm(angle: u32) -> u16 {
    // 안전을 위해 각도를 0-180도 범위로 제한
    let clamped_angle = angle.clamp(0, 180);
    let calib = calibration();
    // SERVO_MIN과 SERVO_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = calib.servo_min as f64
        + (clamped_angle as f64 / 180.0) * (calib.servo_max - calib.servo_min) as f64;
    pwm_value.round() as u16
}

/// DC 모터를 제어하는 함수
///
/// # Arguments
///
/// * `pwm` - Pca9685 인스턴스에 대한 가변 참조
/// * `motor` - 제어할 모터 (Motor 열거형)
/// * `direction` - 회전 방향 (Direction 열거형)
/// * `speed` - 모터 속도 (0 ~ 4095 사이의 값)
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

/// 특정 모터를 정지시키는 함수
pub fn motor_stop(pwm: &mut Pca9685<I2cdev>, motor: Motor) {
    let calib = calibration();
    let (in1, in2) = match motor {
        Motor::M1 => motor_channels(&calib.motor_m1),
        Motor::M2 => motor_channels(&calib.motor_m2),
    };

    let _ = pwm.set_channel_off(in1, 0);
    let _ = pwm.set_channel_off(in2, 0);
}

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

fn motor_channels(calib: &MotorChannelCalibration) -> (Channel, Channel) {
    (calib.in1, calib.in2)
}
