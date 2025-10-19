use std::time::Duration;

#[derive(Clone, Copy, Debug)]
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
