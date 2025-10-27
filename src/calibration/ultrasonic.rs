//! 초음파 센서 동작 주기 및 GPIO 핀 설정을 정의한다.

use std::time::Duration;

#[derive(Clone, Copy, Debug)]
/// HC-SR04 센서를 제어하기 위한 핀/주기 설정.
pub struct UltrasonicCalibration {
    pub trigger_pin: u8,
    pub echo_pin: u8,
    pub log_interval: Duration,
    pub sample_interval: Duration,
}

impl Default for UltrasonicCalibration {
    fn default() -> Self {
        Self {
            trigger_pin: 23,
            echo_pin: 24,
            log_interval: Duration::from_secs(1),
            sample_interval: Duration::from_millis(100),
        }
    }
}
