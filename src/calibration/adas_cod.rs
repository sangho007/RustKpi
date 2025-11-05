//! ADAS 제어 로직에서 사용하는 파라미터 정의.
//! - `AdasLateralCalibration`: 차선 각도 기반 조향 제어 설정.
//! - `AdasLongitudinalCalibration`: 신호/초음파 기반 속도 제어 설정.

use std::time::Duration;

#[derive(Clone, Copy, Debug)]
/// 차선 각도 조향 제어 파라미터.
/// 서보 각도 범위와 비례 제어 게인, 레이트 리밋 값을 포함한다.
pub struct AdasLateralCalibration {
    /// 비례 제어 게인: 서보각(도) = neutral + k * lane_angle(도)
    pub lane_to_servo_gain: f64,
    /// 5차 스무딩 경로 곡률을 서보각으로 변환하는 게인.
    pub curvature_to_servo_gain: f64,
    /// 횡방향 편차 보정 게인.
    pub lateral_offset_gain: f64,
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
            curvature_to_servo_gain: -1.0,
            lateral_offset_gain: -3.0,
            servo_neutral_deg: 90,
            servo_min_deg: 0,
            servo_max_deg: 180,
            servo_channel_index: 0,
            max_servo_delta_deg: 10,
        }
    }
}

#[derive(Clone, Copy, Debug)]
/// 종방향 속도 제어 파라미터.
/// 속도 명령 비율과 거리 임계값, 로깅 주기를 설정한다.
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
    /// 곡률 기반 감속을 적용할 임계값(절대값).
    pub curvature_slowdown_threshold: f64,
    /// 정지 요청이 지속되어야 하는 최소 시간.
    pub stop_request_hold_time: Duration,
    /// 정지 요청 해제 후 평상시로 돌아가기까지 유지할 최소 시간.
    pub stop_release_hold_time: Duration,
    /// 루프당 허용되는 가속 증가량(퍼센트 포인트).
    pub max_accel_delta_percent: u32,
    /// 루프당 허용되는 감속 감소량(퍼센트 포인트).
    pub max_decel_delta_percent: u32,
}

impl Default for AdasLongitudinalCalibration {
    fn default() -> Self {
        Self {
            control_period: Duration::from_millis(50),
            cruise_speed_percent: 50,
            crawl_speed_percent: 35,
            slowdown_distance_cm: 60.0,
            stop_distance_cm: 35.0,
            log_interval: Duration::from_secs(1),
            curvature_slowdown_threshold: 0.015,
            stop_request_hold_time: Duration::from_millis(200),
            stop_release_hold_time: Duration::from_millis(300),
            max_accel_delta_percent: 5,
            max_decel_delta_percent: 12,
        }
    }
}
