//! 차선 각도 추정에 사용하는 1차원 칼만 필터 설정.

#[derive(Clone, Copy, Debug)]
/// 필터 사용 여부와 노이즈 공분산 등을 정의한다.
pub struct KalmanCalibration {
    pub enabled: bool,
    pub process_noise: f64,
    pub measurement_noise: f64,
    pub initial_estimate: f64,
    pub initial_covariance: f64,
}

impl Default for KalmanCalibration {
    fn default() -> Self {
        Self {
            enabled: false,
            process_noise: 0.01,
            measurement_noise: 0.5,
            initial_estimate: 0.0,
            initial_covariance: 1.0,
        }
    }
}
