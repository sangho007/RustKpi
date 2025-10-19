#[derive(Clone, Copy, Debug)]
pub struct ForwardCollisionCalibration {
    pub threshold_distance: f32,
}

impl Default for ForwardCollisionCalibration {
    fn default() -> Self {
        Self {
            threshold_distance: 30.0,
        }
    }
}
