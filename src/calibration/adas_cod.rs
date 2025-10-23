use std::time::Duration;

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
            lane_to_servo_gain: -5.0,
            servo_neutral_deg: 90,
            servo_min_deg: 0,
            servo_max_deg: 180,
            servo_channel_index: 0,
            max_servo_delta_deg: 10,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AdasLongitudinalCalibration {
    /// 제어 루프 주기
    pub control_period: Duration,
    /// 장애물이 없을 때 설정할 순항 속도(퍼센트)
    pub cruise_speed_percent: u32,
    /// 신호등 황색이나 근접 객체가 있을 때 사용할 감속 속도(퍼센트)
    pub crawl_speed_percent: u32,
    /// 감속을 시작할 초음파 거리(cm)
    pub slowdown_distance_cm: f32,
    /// 안전 정지를 위한 최소 거리(cm)
    pub stop_distance_cm: f32,
    /// 상태 로그 주기
    pub log_interval: Duration,
}

impl Default for AdasLongitudinalCalibration {
    fn default() -> Self {
        Self {
            control_period: Duration::from_millis(100),
            cruise_speed_percent: 60,
            crawl_speed_percent: 25,
            slowdown_distance_cm: 60.0,
            stop_distance_cm: 35.0,
            log_interval: Duration::from_secs(1),
        }
    }
}
