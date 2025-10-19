pub mod camera;
pub mod image;
pub mod processing;
pub mod runtime;

use camera::CameraCalibration;
use image::ImageCalibration;
use image::perspective::PerspectiveCalibration;
use image::roi::RoiCalibration;
use processing::ProcessingCalibration;
use processing::filtering::FilteringCalibration;
use processing::kalman::KalmanCalibration;
use processing::morphology::MorphologyCalibration;
use processing::sliding::SlidingWindowCalibration;
use runtime::LaneRuntimeCalibration;

#[derive(Clone, Copy, Debug)]
pub struct LaneCalibration {
    pub camera: CameraCalibration,
    pub image: ImageCalibration,
    pub processing: ProcessingCalibration,
    pub runtime: LaneRuntimeCalibration,
}

#[derive(Clone, Copy, Debug)]
pub enum LaneCalibrationPreset {
    Hd1280x720,
    Vga640x480,
}

impl Default for LaneCalibration {
    fn default() -> Self {
        Self::preset(LaneCalibrationPreset::Vga640x480)
    }
}

impl LaneCalibration {
    pub fn preset(preset: LaneCalibrationPreset) -> Self {
        match preset {
            LaneCalibrationPreset::Hd1280x720 => {
                let camera = CameraCalibration {
                    width: 1280,
                    height: 720,
                };
                let image = ImageCalibration {
                    roi: RoiCalibration::new([(200, 620), (540, 480), (740, 480), (1080, 620)]),
                    perspective: PerspectiveCalibration::new(
                        [
                            (200.0, 620.0),
                            (540.0, 480.0),
                            (740.0, 480.0),
                            (1080.0, 620.0),
                        ],
                        [(200.0, 720.0), (300.0, 0.0), (980.0, 0.0), (1080.0, 720.0)],
                    ),
                };
                let processing = ProcessingCalibration {
                    filtering: FilteringCalibration::default(),
                    morphology: MorphologyCalibration::default(),
                    sliding: SlidingWindowCalibration::default(),
                    kalman: KalmanCalibration::default(),
                };
                let runtime = LaneRuntimeCalibration::default();
                Self {
                    camera,
                    image,
                    processing,
                    runtime,
                }
            }
            LaneCalibrationPreset::Vga640x480 => {
                let camera = CameraCalibration {
                    width: 640,
                    height: 480,
                };

                let width_ratio = camera.width as f32 / 1280.0;
                let height_ratio = camera.height as f32 / 720.0;

                let hd_roi = [(200, 620), (540, 480), (740, 480), (1080, 620)];
                let mut roi_vertices = [(0, 0); 4];
                for (idx, (x, y)) in hd_roi.into_iter().enumerate() {
                    let scaled_x = ((x as f32) * width_ratio)
                        .round()
                        .clamp(0.0, camera.width as f32 - 1.0)
                        as i32;
                    let scaled_y = ((y as f32) * height_ratio)
                        .round()
                        .clamp(0.0, camera.height as f32 - 1.0)
                        as i32;
                    roi_vertices[idx] = (scaled_x, scaled_y);
                }

                let hd_source = [
                    (200.0, 620.0),
                    (540.0, 480.0),
                    (740.0, 480.0),
                    (1080.0, 620.0),
                ];
                let hd_destination = [(200.0, 720.0), (300.0, 0.0), (980.0, 0.0), (1080.0, 720.0)];

                let mut source = [(0.0, 0.0); 4];
                let mut destination = [(0.0, 0.0); 4];
                for (idx, (x, y)) in hd_source.into_iter().enumerate() {
                    source[idx] = (x * width_ratio, y * height_ratio);
                }
                for (idx, (x, y)) in hd_destination.into_iter().enumerate() {
                    destination[idx] = (x * width_ratio, y * height_ratio);
                }

                let image = ImageCalibration {
                    roi: RoiCalibration::new(roi_vertices),
                    perspective: PerspectiveCalibration::new(source, destination),
                };

                let processing = ProcessingCalibration {
                    filtering: FilteringCalibration::default(),
                    morphology: MorphologyCalibration::default(),
                    sliding: SlidingWindowCalibration {
                        display_margin: 240,
                        search_margin: 80,
                        window_count: 15,
                        minpix: 30,
                        required_points: 2500,
                        draw_debug_windows: true,
                        search_poly_margin: 40,
                    },
                    kalman: KalmanCalibration::default(),
                };

                let runtime = LaneRuntimeCalibration::default();

                Self {
                    camera,
                    image,
                    processing,
                    runtime,
                }
            }
        }
    }

    pub fn for_dimensions(width: i32, height: i32) -> Self {
        match (width, height) {
            (1280, 720) => Self::preset(LaneCalibrationPreset::Hd1280x720),
            (640, 480) => Self::preset(LaneCalibrationPreset::Vga640x480),
            _ => Self::default(),
        }
    }
}
