//! 초음파 전방 충돌 감지용 캘리브레이션 헬퍼.
//! 러너블에서 손쉽게 기본 설정을 로드할 수 있도록 추상화한다.

use crate::calibration::forward_collision::ForwardCollisionCalibration;

/// 전방 충돌 감지 캘리브레이션을 반환한다.
pub fn forward_collision_calibration() -> ForwardCollisionCalibration {
    ForwardCollisionCalibration::default()
}
