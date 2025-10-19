#[derive(Clone, Copy, Debug)]
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
