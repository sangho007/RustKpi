pub mod perspective;
pub mod roi;

use perspective::PerspectiveCalibration;
use roi::RoiCalibration;

#[derive(Clone, Copy, Debug)]
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
