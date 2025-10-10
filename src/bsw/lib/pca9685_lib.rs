use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Channel, Pca9685};

// --- 상수 정의 ---
pub const I2C_BUS: &str = "/dev/i2c-1";
pub const PCA9685_ADDRESS: u8 = 0x5f; // I2C 주소

// 서보 모터 3개 채널
pub const SERVO_CHANNELS: [Channel; 3] = [Channel::C0, Channel::C1, Channel::C2];

// Dc 모터  채널
pub const MOTOR_M1_IN1: Channel = Channel::C15;
pub const MOTOR_M1_IN2: Channel = Channel::C14;
pub const MOTOR_M2_IN1: Channel = Channel::C12;
pub const MOTOR_M2_IN2: Channel = Channel::C13;

// 0-180도에 대한 서보 펄스 폭 범위
pub const SERVO_MIN: u16 = 205; // 0도일 때 약 1ms 펄스
pub const SERVO_MAX: u16 = 410; // 180도일 때 약 2ms 펄스

// 0-100% 에 대한 DC 펄스 폭 범위
pub const DC_MIN: u16 = 0;
pub const DC_MAX: u16 = 4096;



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

/// 원하는 각도(0-180)를 PCA9685의 PWM off-cycle 값으로 변환합니다.
pub fn angle_to_pwm(angle: u32) -> u16 {
    // 안전을 위해 각도를 0-180도 범위로 제한
    let clamped_angle = angle.clamp(0, 180);
    // SERVO_MIN과 SERVO_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = SERVO_MIN as f64 + (clamped_angle as f64 / 180.0) * (SERVO_MAX - SERVO_MIN) as f64;
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
pub fn motor_control(
    pwm: &mut Pca9685<I2cdev>,
    motor: Motor,
    direction: Direction,
    speed: u16,
) {
    let (in1, in2) = match motor {
        Motor::M1 => (MOTOR_M1_IN1, MOTOR_M1_IN2),
        Motor::M2 => (MOTOR_M2_IN1, MOTOR_M2_IN2),
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
pub fn motor_stop(
    pwm: &mut Pca9685<I2cdev>,
    motor: Motor,
) {
    let (in1, in2) = match motor {
        Motor::M1 => (MOTOR_M1_IN1, MOTOR_M1_IN2),
        Motor::M2 => (MOTOR_M2_IN1, MOTOR_M2_IN2),
    };

    let _ = pwm.set_channel_off(in1, 0);
    let _ = pwm.set_channel_off(in2, 0);
}

pub fn percent_to_pwm(angle: u32) -> u16 {
    // 안전을 위해 각도를 0-180도 범위로 제한
    let clamped_angle = angle.clamp(0, 100);
    // SERVO_MIN과 SERVO_MAX 사이를 선형 보간하여 PWM 값을 계산
    let pwm_value = SERVO_MIN as f64 + (clamped_angle as f64 / 100.0) * (SERVO_MAX - SERVO_MIN) as f64;
    pwm_value.round() as u16
}
