pub mod filtering;
pub mod kalman;
pub mod morphology;
pub mod sliding;

use filtering::FilteringCalibration;
use kalman::KalmanCalibration;
use morphology::MorphologyCalibration;
use sliding::SlidingWindowCalibration;

#[derive(Clone, Copy, Debug)]
pub struct ProcessingCalibration {
    pub filtering: FilteringCalibration,
    pub morphology: MorphologyCalibration,
    pub sliding: SlidingWindowCalibration,
    pub kalman: KalmanCalibration,
}

impl Default for ProcessingCalibration {
    fn default() -> Self {
        Self {
            filtering: FilteringCalibration::default(),
            morphology: MorphologyCalibration::default(),
            sliding: SlidingWindowCalibration::default(),
            kalman: KalmanCalibration::default(),
        }
    }
}
