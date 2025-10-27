//! 차선 검출 파이프라인에 필요한 캘리브레이션 묶음.
//! 카메라/이미지/처리/런타임 설정을 프리셋 형태로 제공한다.

pub mod camera;
pub mod image;
pub mod processing;
pub mod runtime;

pub use camera::CameraCalibration;
pub use image::{ImageCalibration, perspective::PerspectiveCalibration, roi::RoiCalibration};
pub use processing::{
    ProcessingCalibration, filtering::FilteringCalibration, kalman::KalmanCalibration,
    morphology::MorphologyCalibration, sliding::SlidingWindowCalibration,
};
pub use runtime::LaneRuntimeCalibration;

#[derive(Clone, Copy, Debug)]
/// 차선 검출에 필요한 모든 하위 설정을 묶은 구조체.
pub struct LaneCalibration {
    pub camera: CameraCalibration,
    pub image: ImageCalibration,
    pub processing: ProcessingCalibration,
    pub runtime: LaneRuntimeCalibration,
}

#[derive(Clone, Copy, Debug)]
/// 지원하는 캘리브레이션 프리셋 목록.
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
    /// 사전 정의된 프리셋으로 캘리브레이션을 구성한다.
    pub fn preset(preset: LaneCalibrationPreset) -> Self {
        match preset {
            LaneCalibrationPreset::Hd1280x720 => {
                let camera = CameraCalibration {
                    width: 1280,
                    height: 720,
                    target_fps: 30,
                    capture_queue_depth: 3,
                    use_libcamera: true,
                    sample_video_preferred: "./video/challenge_1280x720.mp4",
                    sample_video_fallback: "./video/challenge.mp4",
                };
                let base_camera = CameraCalibration::default();
                let width_ratio = camera.width as f32 / base_camera.width as f32;
                let height_ratio = camera.height as f32 / base_camera.height as f32;

                let mut roi_vertices = RoiCalibration::default().vertices;
                for (x, y) in roi_vertices.iter_mut() {
                    *x = ((*x as f32) * width_ratio)
                        .round()
                        .clamp(0.0, camera.width as f32 - 1.0) as i32;
                    *y = ((*y as f32) * height_ratio)
                        .round()
                        .clamp(0.0, camera.height as f32 - 1.0) as i32;
                }

                let mut source = PerspectiveCalibration::default().source;
                for point in source.iter_mut() {
                    point.0 *= width_ratio;
                    point.1 *= height_ratio;
                }

                let mut destination = PerspectiveCalibration::default().destination;
                for point in destination.iter_mut() {
                    point.0 *= width_ratio;
                    point.1 *= height_ratio;
                }

                let image = ImageCalibration {
                    roi: RoiCalibration::new(roi_vertices),
                    perspective: PerspectiveCalibration::new(source, destination),
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
                let mut camera = CameraCalibration::default();
                camera.use_libcamera = false;

                let image = ImageCalibration::default();

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

    /// 입력 해상도에 맞는 프리셋을 자동으로 선택한다.
    pub fn for_dimensions(width: i32, height: i32) -> Self {
        match (width, height) {
            (1280, 720) => Self::preset(LaneCalibrationPreset::Hd1280x720),
            (640, 480) => Self::preset(LaneCalibrationPreset::Vga640x480),
            _ => Self::default(),
        }
    }
}
