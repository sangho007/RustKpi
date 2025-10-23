use crate::calibration::camera::CameraCalibration;

const BASE_WIDTH: f32 = 640.0;
const BASE_HEIGHT: f32 = 480.0;
const BASE_ROI: [(i32, i32); 4] = [(100, 413), (270, 320), (370, 320), (540, 413)];

#[derive(Clone, Copy, Debug)]
pub struct TrafficLightColorThreshold {
    pub lower: (u8, u8, u8),
    pub upper: (u8, u8, u8),
}

#[derive(Clone, Copy, Debug)]
pub struct TrafficLightCalibration {
    pub detection_interval: u32,
    pub min_pixel_threshold: usize,
    pub dbscan_epsilon: f64,
    pub dbscan_min_points: usize,
    pub frame_width: i32,
    pub frame_height: i32,
    pub roi_vertices: [(i32, i32); 4],
    pub red_threshold: TrafficLightColorThreshold,
    pub yellow_threshold: TrafficLightColorThreshold,
    pub green_threshold: TrafficLightColorThreshold,
}

impl Default for TrafficLightCalibration {
    fn default() -> Self {
        let camera = CameraCalibration::default();
        let width = camera.width;
        let height = camera.height;
        let width_ratio = width as f32 / BASE_WIDTH;
        let height_ratio = height as f32 / BASE_HEIGHT;
        let roi_vertices = BASE_ROI.map(|(x, y)| {
            let scaled_x = ((x as f32) * width_ratio)
                .round()
                .max(0.0)
                .min((width - 1) as f32) as i32;
            let scaled_y = ((y as f32) * height_ratio)
                .round()
                .max(0.0)
                .min((height - 1) as f32) as i32;
            (scaled_x, scaled_y)
        });

        Self {
            detection_interval: 5,
            min_pixel_threshold: 100,
            dbscan_epsilon: 20.0,
            dbscan_min_points: 15,
            frame_width: width,
            frame_height: height,
            roi_vertices,
            red_threshold: TrafficLightColorThreshold {
                lower: (0, 120, 70),
                upper: (10, 255, 255),
            },
            yellow_threshold: TrafficLightColorThreshold {
                lower: (20, 100, 100),
                upper: (30, 255, 255),
            },
            green_threshold: TrafficLightColorThreshold {
                lower: (50, 100, 100),
                upper: (70, 255, 255),
            },
        }
    }
}
