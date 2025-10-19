pub mod camera;
pub mod image;
pub mod processing;
pub mod runtime;

use camera::CameraCalibration;
use image::ImageCalibration;
use processing::ProcessingCalibration;
use runtime::LaneRuntimeCalibration;

#[derive(Clone, Copy, Debug)]
pub struct LaneCalibration {
    pub camera: CameraCalibration,
    pub image: ImageCalibration,
    pub processing: ProcessingCalibration,
    pub runtime: LaneRuntimeCalibration,
}

impl Default for LaneCalibration {
    fn default() -> Self {
        Self {
            camera: CameraCalibration::default(),
            image: ImageCalibration::default(),
            processing: ProcessingCalibration::default(),
            runtime: LaneRuntimeCalibration::default(),
        }
    }
}
