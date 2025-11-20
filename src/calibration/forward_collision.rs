//! 전방 충돌 감지 임계값 캘리브레이션.

#[derive(Clone, Copy, Debug)]
/// 초음파 거리 기반 전방 충돌 대응 임계값 묶음(단위: cm).
pub struct ForwardCollisionCalibration {
    /// 정지 판단 기준 거리. 이 이하이면 즉시 정지 요청.
    pub stop_request_distance_cm: f32,
    /// 차선 변경 유도 기준 거리. 이 이하이면 차선 변경을 권고한다.
    pub lane_change_request_distance_cm: f32,
}

impl Default for ForwardCollisionCalibration {
    fn default() -> Self {
        Self {
            stop_request_distance_cm: 20.0,
            lane_change_request_distance_cm: 0.0,
        }
    }
}
