use pwm_pca9685::Channel;
use std::time::Duration;

#[derive(Clone, Copy, Debug)]
pub struct MotorChannelCalibration {
    pub in1: Channel,
    pub in2: Channel,
}

#[derive(Clone, Copy, Debug)]
pub struct PwmCalibration {
    pub i2c_bus: &'static str,
    pub device_address: u8,
    pub servo_channels: [Channel; 3],
    pub servo_default_angles: [u32; 3],
    pub motor_m1: MotorChannelCalibration,
    pub motor_m2: MotorChannelCalibration,
    pub servo_min: u16,
    pub servo_max: u16,
    pub dc_min: u16,
    pub dc_max: u16,
    pub servo_log_interval: Duration,
    pub dc_log_interval: Duration,
}

impl Default for PwmCalibration {
    fn default() -> Self {
        Self {
            i2c_bus: "/dev/i2c-1",
            device_address: 0x5f,
            servo_channels: [Channel::C0, Channel::C1, Channel::C2],
            servo_default_angles: [90, 180, 170],
            motor_m1: MotorChannelCalibration {
                in1: Channel::C15,
                in2: Channel::C14,
            },
            motor_m2: MotorChannelCalibration {
                in1: Channel::C12,
                in2: Channel::C13,
            },
            servo_min: 205,
            servo_max: 410,
            dc_min: 0,
            dc_max: 4096,
            servo_log_interval: Duration::from_secs(1),
            dc_log_interval: Duration::from_secs(1),
        }
    }
}
