//! 모폴로지 연산(폐연산 등) 파라미터.

#[derive(Clone, Copy, Debug)]
/// 구조 요소 크기와 반복 횟수 설정.
pub struct MorphologyCalibration {
    pub kernel_size: (i32, i32),
    pub iterations: i32,
}

impl Default for MorphologyCalibration {
    fn default() -> Self {
        Self {
            kernel_size: (3, 3),
            iterations: 1,
        }
    }
}
