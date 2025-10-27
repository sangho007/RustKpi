//! 차선 처리 파이프라인의 세부 캘리브레이션.
//! 필터링, 모폴로지, 슬라이딩 윈도우, 칼만 필터를 구성한다.

pub mod filtering;
pub mod kalman;
pub mod morphology;
pub mod sliding;

use filtering::FilteringCalibration;
use kalman::KalmanCalibration;
use morphology::MorphologyCalibration;
use sliding::SlidingWindowCalibration;

#[derive(Clone, Copy, Debug)]
/// 영상 처리 파이프라인 전반의 설정 묶음.
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
