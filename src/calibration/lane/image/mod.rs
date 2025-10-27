//! 이미지 레벨에서의 ROI/투시 변환 캘리브레이션.

pub mod perspective;
pub mod roi;

use perspective::PerspectiveCalibration;
use roi::RoiCalibration;

#[derive(Clone, Copy, Debug)]
/// 관심 영역과 투시 변환 행렬을 묶은 구조체.
pub struct ImageCalibration {
    pub roi: RoiCalibration,
    pub perspective: PerspectiveCalibration,
}

impl Default for ImageCalibration {
    fn default() -> Self {
        Self {
            roi: RoiCalibration::default(),
            perspective: PerspectiveCalibration::default(),
        }
    }
}
