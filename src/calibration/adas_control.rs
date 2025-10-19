#[derive(Clone, Copy, Debug)]
pub struct AdasLateralCalibration {
    /// 비례 제어 게인: 서보각(도) = neutral + k * lane_angle(도)
    pub lane_to_servo_gain: f64,
    /// 서보 중립 각도(도)
    pub servo_neutral_deg: u32,
    /// 서보 최소/최대 각도(도)
    pub servo_min_deg: u32,
    pub servo_max_deg: u32,
    /// 제어할 서보 채널 인덱스 (PwmCalibration.servo_channels 배열의 인덱스)
    pub servo_channel_index: u8,
    /// 루프당 최대 서보 각 변화량(도)
    pub max_servo_delta_deg: u32,
}

impl Default for AdasLateralCalibration {
    fn default() -> Self {
        Self {
            lane_to_servo_gain: 1.5,
            servo_neutral_deg: 90,
            servo_min_deg: 0,
            servo_max_deg: 180,
            servo_channel_index: 0,
            max_servo_delta_deg: 5,
        }
    }
}

