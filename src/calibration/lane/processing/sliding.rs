//! 슬라이딩 윈도우 기반 차선 탐색 파라미터.

#[derive(Clone, Copy, Debug)]
/// 윈도우 개수, 탐색 범위, 디버그 옵션 등을 설정한다.
pub struct SlidingWindowCalibration {
    pub display_margin: i32,
    pub search_margin: i32,
    pub window_count: i32,
    pub minpix: i32,
    pub required_points: usize,
    pub draw_debug_windows: bool,
    pub search_poly_margin: i32,
}

impl Default for SlidingWindowCalibration {
    fn default() -> Self {
        Self {
            display_margin: 480,
            search_margin: 150,
            window_count: 15,
            minpix: 50,
            required_points: 5000,
            draw_debug_windows: true,
            search_poly_margin: 80,
        }
    }
}
