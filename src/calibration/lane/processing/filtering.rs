//! 영상 전처리(블러, 캐니 엣지) 파라미터.

#[derive(Clone, Copy, Debug)]
/// 가우시안 블러와 캐니 엣지 설정값.
pub struct FilteringCalibration {
    pub gaussian_kernel: (i32, i32),
    pub gaussian_sigma: (f64, f64),
    pub canny_low_threshold: f64,
    pub canny_high_threshold: f64,
    pub canny_aperture_size: i32,
}

impl Default for FilteringCalibration {
    fn default() -> Self {
        Self {
            gaussian_kernel: (5, 5),
            gaussian_sigma: (0.0, 0.0),
            canny_low_threshold: 200.0,
            canny_high_threshold: 350.0,
            canny_aperture_size: 3,
        }
    }
}
