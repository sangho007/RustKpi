#[derive(Clone, Copy, Debug)]
pub struct CameraCalibration {
    pub width: i32,
    pub height: i32,
}

impl Default for CameraCalibration {
    fn default() -> Self {
        Self {
            width: 1280,
            height: 720,
        }
    }
}
