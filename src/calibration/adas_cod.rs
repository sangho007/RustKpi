//! ADAS 제어 로직에서 사용하는 파라미터 정의.
//! - `AdasLateralCalibration`: 스무딩 샘플 횡오차 PID 조향 제어 설정.
//! - `AdasLongitudinalCalibration`: 신호/초음파 기반 속도 제어 설정.

use std::env;
use std::time::Duration;

#[derive(Clone, Copy, Debug)]
/// 차선 각도 조향 제어 파라미터.
/// 서보 각도 범위와 비례 제어 게인, 레이트 리밋 값을 포함한다.
pub struct AdasLateralCalibration {
    /// PID 비례 게인 (deg 명령 / m lateral error)
    pub pid_kp: f64,
    /// PID 적분 게인 (deg 명령 / (m·s))
    pub pid_ki: f64,
    /// PID 미분 게인 (deg 명령·s / m)
    pub pid_kd: f64,
    /// 적분 항 누적 한계 (m·s 단위)
    pub pid_integral_limit: f64,
    /// 횡오차 계산에 사용할 스무딩 샘플 인덱스(0-base)
    pub pid_sample_index: usize,
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
        Self::baseline()
    }
}

impl AdasLateralCalibration {
    fn baseline() -> Self {
        Self {
            pid_kp: 150.0,
            pid_ki: 0.0,
            pid_kd: 0.0,
            pid_integral_limit: 0.5,
            pid_sample_index: 4,
            servo_neutral_deg: 90,
            servo_min_deg: 0,
            servo_max_deg: 180,
            servo_channel_index: 0,
            max_servo_delta_deg: 30,
        }
    }

    pub fn from_env() -> Self {
        let mut calib = Self::baseline();
        if let Some(value) = read_env_f64("ADAS_PID_KP") {
            calib.pid_kp = value;
        }
        if let Some(value) = read_env_f64("ADAS_PID_KI") {
            calib.pid_ki = value;
        }
        if let Some(value) = read_env_f64("ADAS_PID_KD") {
            calib.pid_kd = value;
        }
        calib
    }
}

fn read_env_f64(key: &str) -> Option<f64> {
    match env::var(key) {
        Ok(raw) => match raw.trim().parse::<f64>() {
            Ok(value) => Some(value),
            Err(err) => {
                eprintln!("[Calib] {} parse 실패: {} (입력='{}')", key, err, raw);
                None
            }
        },
        Err(env::VarError::NotPresent) => None,
        Err(err) => {
            eprintln!("[Calib] {} 읽기 실패: {}", key, err);
            None
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
    /// 기본 목표 속도(m/s). 20cm/s를 지향한다.
    pub speed_target_mps: f64,
    /// 속도 PID 비례 게인 (퍼센트 / m/s)
    pub speed_pid_kp: f64,
    /// 속도 PID 적분 게인
    pub speed_pid_ki: f64,
    /// 속도 PID 미분 게인
    pub speed_pid_kd: f64,
    /// 속도 PID 적분 항 한계
    pub speed_pid_integral_limit: f64,
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
            //cruise_speed_percent: 0,
            //crawl_speed_percent: 0,
            cruise_speed_percent: 30,
            crawl_speed_percent: 20,
            speed_target_mps: 0.25,
            speed_pid_kp: 30.0,
            speed_pid_ki: 0.5,
            speed_pid_kd: 8.0,
            speed_pid_integral_limit: 0.4,
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
