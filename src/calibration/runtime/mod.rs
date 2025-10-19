#[derive(Clone, Copy, Debug)]
pub struct LaneRuntimeCalibration {
    pub process_interval: u32,
}

impl Default for LaneRuntimeCalibration {
    fn default() -> Self {
        Self {
            process_interval: 3,
        }
    }
}
