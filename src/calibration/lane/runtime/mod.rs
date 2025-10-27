//! 차선 파이프라인의 런타임 제어 파라미터.

#[derive(Clone, Copy, Debug)]
/// 프레임 건너뛰기 간격 등 런타임 스케줄링 값.
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
