//! 신호등 인식에 필요한 캘리브레이션 값.
//! ROI 스케일링과 HSV 색상 임계값, 클러스터링 파라미터를 정의한다.

use crate::calibration::adas_localization::LocalizationMapId;
use crate::calibration::camera::CameraCalibration;

const BASE_WIDTH: f32 = 640.0;
const BASE_HEIGHT: f32 = 480.0;
const BASE_ROI: [(i32, i32); 4] = [(100, 413), (270, 320), (370, 320), (540, 413)];

#[derive(Clone, Copy, Debug)]
/// HSV 색 공간에서 특정 색을 선택하기 위한 하한/상한 값.
pub struct TrafficLightColorThreshold {
    pub lower: (u8, u8, u8),
    pub upper: (u8, u8, u8),
}

#[derive(Clone, Copy, Debug)]
/// 신호등 인식 전체 설정.
/// ROI 좌표와 DBSCAN 파라미터, 색상 임계값을 포함한다.
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

#[derive(Clone, Copy, Debug)]
/// 신호등 감지 활성화 구간(맵 좌표계 기준, 단위: m).
pub struct TrafficLightDetectionZone {
    pub map: LocalizationMapId,
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

/// 신호등 감지에 사용할 맵/좌표 구간 목록.
pub const TRAFFIC_LIGHT_DETECTION_ZONES: &[TrafficLightDetectionZone] =
    &[TrafficLightDetectionZone {
        map: LocalizationMapId::Crossroad,
        x_min: -0.3,
        x_max: 0.3,
        y_min: -1.24,
        y_max: -0.8,
    }];
