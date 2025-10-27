//! 전방 충돌 감지 임계값 캘리브레이션.

#[derive(Clone, Copy, Debug)]
/// 초음파 거리 기준으로 장애물을 판단하는 임계값.
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
