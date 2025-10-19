#[derive(Clone, Copy, Debug)]
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
