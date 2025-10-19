// 필요한 모듈들을 use 키워드로 가져옵니다.
use dbscan::*;
use opencv::{
    Result,
    core::{self, AlgorithmHint::ALGO_HINT_DEFAULT, Mat, Point, Scalar, Size}, // Size 추가
    imgproc,
    prelude::*,
};
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
pub enum TrafficLightColor {
    Red,
    Yellow,
    Green,
    Off, // 소등 상태
}

// 클러스터의 크기를 기반으로 탐지할 최소 픽셀 수
const MIN_PIXEL_THRESHOLD: usize = 100;

// epsilon: 같은 클러스터로 간주할 최대 거리 (픽셀 단위)
// min_points: 클러스터를 형성하기 위한 최소 점의 개수
const EPSILON: f64 = 20.0;
const MIN_POINTS: usize = 15;

pub struct Pipeline {
    width: i32,
    height: i32,
    vertices: Vec<Point>,
    pub red_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub yellow_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub green_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub current_traffic_light_color: TrafficLightColor,
}

impl Pipeline {
    pub fn new() -> Self {
        let width = 1280;
        let height = 720;

        let vertices = vec![
            Point::new(200, height - 100),
            Point::new(width / 2 - 100, height / 2 + 120),
            Point::new(width / 2 + 100, height / 2 + 120),
            Point::new(width - 200, height - 100),
        ];

        let red_threshold = ((0, 120, 70), (10, 255, 255));
        let yellow_threshold = ((20, 100, 100), (30, 255, 255));
        let green_threshold = ((50, 100, 100), (70, 255, 255));
        let current_traffic_light_color = TrafficLightColor::Off;

        Self {
            width,
            height,
            vertices,
            red_threshold,
            yellow_threshold,
            green_threshold,
            current_traffic_light_color,
        }
    }

    pub fn convert_to_hsv(&self, bgr_frame: &Mat) -> Result<Mat> {
        let mut hsv_frame = Mat::default();
        imgproc::cvt_color(
            bgr_frame,
            &mut hsv_frame,
            imgproc::COLOR_BGR2HSV,
            0,
            ALGO_HINT_DEFAULT,
        )?;
        Ok(hsv_frame)
    }

    /// HSV 프레임에서 신호등 색상을 감지하고 내부 상태를 업데이트합니다.
    pub fn detect_color_from_hsv(&mut self, hsv_frame: &Mat) -> TrafficLightColor {
        // 각 색상에 대한 마스크를 생성합니다.
        let red_mask = self
            .create_mask(hsv_frame, self.red_threshold)
            .unwrap_or_default();
        let yellow_mask = self
            .create_mask(hsv_frame, self.yellow_threshold)
            .unwrap_or_default();
        let green_mask = self
            .create_mask(hsv_frame, self.green_threshold)
            .unwrap_or_default();

        // --- ⬇️ 모폴로지 연산으로 노이즈 제거 ⬇️ ---
        let red_mask_denoised = self.apply_morphology(&red_mask).unwrap_or(red_mask);
        let yellow_mask_denoised = self.apply_morphology(&yellow_mask).unwrap_or(yellow_mask);
        let green_mask_denoised = self.apply_morphology(&green_mask).unwrap_or(green_mask);

        let red_pixels = self.find_largest_cluster(&red_mask_denoised);
        let yellow_pixels = self.find_largest_cluster(&yellow_mask_denoised);
        let green_pixels = self.find_largest_cluster(&green_mask_denoised);

        let detected_color = if red_pixels > MIN_PIXEL_THRESHOLD
            && red_pixels >= yellow_pixels
            && red_pixels >= green_pixels
        {
            TrafficLightColor::Red
        } else if yellow_pixels > MIN_PIXEL_THRESHOLD
            && yellow_pixels >= red_pixels
            && yellow_pixels >= green_pixels
        {
            TrafficLightColor::Yellow
        } else if green_pixels > MIN_PIXEL_THRESHOLD
            && green_pixels >= red_pixels
            && green_pixels >= yellow_pixels
        {
            TrafficLightColor::Green
        } else {
            TrafficLightColor::Off
        };

        self.current_traffic_light_color = detected_color.clone();
        detected_color
    }

    fn create_mask(&self, hsv_frame: &Mat, threshold: ((u8, u8, u8), (u8, u8, u8))) -> Result<Mat> {
        let mut mask = Mat::default();
        let (lower, upper) = threshold;
        let lower_bound = Scalar::new(lower.0 as f64, lower.1 as f64, lower.2 as f64, 0.0);
        let upper_bound = Scalar::new(upper.0 as f64, upper.1 as f64, upper.2 as f64, 0.0);
        core::in_range(hsv_frame, &lower_bound, &upper_bound, &mut mask)?;
        Ok(mask)
    }

    /// 모폴로지 열림(Opening) 연산을 적용하여 마스크의 노이즈를 제거합니다.
    ///
    /// # Arguments
    /// * `mask` - 노이즈를 제거할 바이너리 마스크 (`&Mat`)
    ///
    /// # Returns
    /// 노이즈가 제거된 마스크 `Mat`을 포함하는 `Result`를 반환합니다.
    fn apply_morphology(&self, mask: &Mat) -> Result<Mat> {
        let mut processed_mask = Mat::default();
        // 연산의 강도를 결정하는 커널(구조 요소) 생성. 크기를 조절하여 효과를 변경할 수 있습니다.
        let kernel = imgproc::get_structuring_element(
            imgproc::MORPH_ELLIPSE, // 타원형 커널이 신호등 같은 둥근 객체에 유리
            Size::new(5, 5),
            Point::new(-1, -1),
        )?;

        // 열림(Opening) 연산 (침식 -> 팽창)을 적용하여 작은 노이즈 객체들을 제거합니다.
        imgproc::morphology_ex(
            mask,
            &mut processed_mask,
            imgproc::MORPH_OPEN,
            &kernel,
            Point::new(-1, -1),
            1, // 반복 횟수
            core::BORDER_CONSTANT,
            imgproc::morphology_default_border_value()?,
        )?;

        Ok(processed_mask)
    }

    /// 마스크 이미지에서 가장 큰 픽셀 클러스터의 크기를 찾습니다.
    ///
    /// # Arguments
    /// * `mask` - DBSCAN 클러스터링을 적용할 바이너리 마스크 (`&Mat`)
    ///
    /// # Returns
    /// 가장 큰 클러스터에 속한 픽셀의 개수(`usize`)를 반환합니다. 클러스터가 없으면 0을 반환합니다.
    fn find_largest_cluster(&self, mask: &Mat) -> usize {
        // 0이 아닌 픽셀의 좌표를 찾습니다.
        let mut points: Vec<[f64; 2]> = Vec::new();
        for r in 0..mask.rows() {
            for c in 0..mask.cols() {
                if let Ok(pixel_value) = mask.at_2d::<u8>(r, c) {
                    if *pixel_value != 0 {
                        // DBSCAN 라이브러리를 위해 [x, y] 좌표를 f64 타입으로 변환
                        points.push([c as f64, r as f64]);
                    }
                }
            }
        }

        if points.is_empty() {
            return 0;
        }

        // DBSCAN 모델을 설정하고 실행합니다.
        let mut model = dbscan::Model::new(EPSILON, MIN_POINTS);

        let points_as_vecs: Vec<Vec<f64>> = points.into_iter().map(|p| p.to_vec()).collect();
        let result = model.run(&points_as_vecs);

        // 각 클러스터의 크기를 계산합니다.
        let mut cluster_counts: HashMap<usize, usize> = HashMap::new();
        for classification in result {
            match classification {
                Classification::Core(cluster_id) | Classification::Edge(cluster_id) => {
                    *cluster_counts.entry(cluster_id).or_insert(0) += 1;
                }
                Classification::Noise => {
                    // 노이즈는 무시
                }
            }
        }

        // 가장 큰 클러스터의 크기를 찾습니다.
        cluster_counts.into_values().max().unwrap_or(0)
    }
}
