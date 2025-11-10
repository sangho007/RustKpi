//! 신호등 인식 파이프라인 구현.
//! - HSV 색 공간과 DBSCAN 클러스터링을 이용해 신호등 색상을 안정적으로 판단한다.
//! - 캘리브레이션 값(ROI, 색상 임계값, 클러스터 파라미터)을 기반으로 동작한다.

use crate::calibration::traffic_light::{TrafficLightCalibration, TrafficLightColorThreshold};
use dbscan::*;
use opencv::{
    Result,
    core::{self, AlgorithmHint::ALGO_HINT_DEFAULT, Mat, Point, Scalar, Size}, // Size 추가
    imgproc,
    prelude::*,
};
use opencv::core::Vector;
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
/// 신호등의 판별 상태를 표현하는 열거형.
/// - `Off`는 소등 또는 인식 실패를 의미한다.
pub enum TrafficLightColor {
    Red,
    Yellow,
    Green,
    Off,
}

/// 신호등 인식 전체 과정을 캡슐화한 파이프라인 구조체.
/// 캘리브레이션 정보와 내부 임계값, DBSCAN 파라미터를 보관한다.
pub struct Pipeline {
    vertices: Vec<Point>,
    pub red_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub yellow_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub green_threshold: ((u8, u8, u8), (u8, u8, u8)),
    pub current_traffic_light_color: TrafficLightColor,
    min_pixel_threshold: usize,
    dbscan_epsilon: f64,
    dbscan_min_points: usize,
}

impl Pipeline {
    /// 캘리브레이션 정보를 기반으로 파이프라인을 초기화한다.
    /// - ROI 정점과 색상 임계값, DBSCAN 파라미터를 내부 상태로 저장한다.
    pub fn new(calibration: TrafficLightCalibration) -> Self {
        let vertices = calibration
            .roi_vertices
            .iter()
            .map(|&(x, y)| Point::new(x, y))
            .collect::<Vec<_>>();

        let red_threshold = thresholds_to_tuple(calibration.red_threshold);
        let yellow_threshold = thresholds_to_tuple(calibration.yellow_threshold);
        let green_threshold = thresholds_to_tuple(calibration.green_threshold);

        Self {
            vertices,
            red_threshold,
            yellow_threshold,
            green_threshold,
            current_traffic_light_color: TrafficLightColor::Off,
            min_pixel_threshold: calibration.min_pixel_threshold,
            dbscan_epsilon: calibration.dbscan_epsilon,
            dbscan_min_points: calibration.dbscan_min_points,
        }
    }

    /// BGR 영상을 HSV 색 공간으로 변환한다.
    /// - OpenCV `cvt_color`를 사용하며, 파이프라인에서 반복적으로 호출된다.
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
    /// 연산 부하를 줄이기 위해 320x240으로 다운스케일한 프레임을 사용해 마스크/클러스터링을 수행합니다.
    pub fn detect_color_from_hsv(&mut self, hsv_frame: &Mat) -> TrafficLightColor {
        // 1) 프레임 다운스케일 (기본 320x240). INTER_AREA로 축소 품질 유지.
        let orig_w = hsv_frame.cols().max(1);
        let orig_h = hsv_frame.rows().max(1);
        let target_w = 320;
        let target_h = 240;
        let mut hsv_small = Mat::default();
        let (scale_x, scale_y) = (target_w as f64 / orig_w as f64, target_h as f64 / orig_h as f64);
        let mut scale = scale_x.min(scale_y);
        if orig_w <= target_w || orig_h <= target_h {
            // 원본이 더 작거나 같은 경우에는 업스케일하지 않고 원본을 그대로 사용
            hsv_small = hsv_frame.clone();
            scale = 1.0;
        } else {
            // dsize 지정으로 리사이즈 수행
            let _ = imgproc::resize(
                hsv_frame,
                &mut hsv_small,
                Size::new(target_w, target_h),
                0.0,
                0.0,
                imgproc::INTER_AREA,
            );
        }

        // 2) 각 색상에 대한 마스크 생성 (다운스케일 이미지 기준)
        let red_mask = self
            .create_mask(&hsv_small, self.red_threshold)
            .unwrap_or_default();
        let yellow_mask = self
            .create_mask(&hsv_small, self.yellow_threshold)
            .unwrap_or_default();
        let green_mask = self
            .create_mask(&hsv_small, self.green_threshold)
            .unwrap_or_default();

        // --- ⬇️ 모폴로지 연산으로 노이즈 제거 ⬇️ ---
        let red_mask_denoised = self.apply_morphology(&red_mask).unwrap_or(red_mask);
        let yellow_mask_denoised = self.apply_morphology(&yellow_mask).unwrap_or(yellow_mask);
        let green_mask_denoised = self.apply_morphology(&green_mask).unwrap_or(green_mask);

        // 3) 해상도 축소에 맞춰 DBSCAN 파라미터를 스케일링한다.
        //    - epsilon은 길이 비례로, min_points는 면적(=scale^2) 비례로 조정.
        let eps_scaled = (self.dbscan_epsilon * scale).max(1.0);
        let min_points_scaled = ((self.dbscan_min_points as f64) * scale * scale)
            .round()
            .max(1.0) as usize;

        let red_pixels =
            self.find_largest_cluster_with(&red_mask_denoised, eps_scaled, min_points_scaled);
        let yellow_pixels =
            self.find_largest_cluster_with(&yellow_mask_denoised, eps_scaled, min_points_scaled);
        let green_pixels =
            self.find_largest_cluster_with(&green_mask_denoised, eps_scaled, min_points_scaled);

        let detected_color = if red_pixels > self.min_pixel_threshold
            && red_pixels >= yellow_pixels
            && red_pixels >= green_pixels
        {
            TrafficLightColor::Red
        } else if yellow_pixels > self.min_pixel_threshold
            && yellow_pixels >= red_pixels
            && yellow_pixels >= green_pixels
        {
            TrafficLightColor::Yellow
        } else if green_pixels > self.min_pixel_threshold
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

    /// 지정된 HSV 임계값으로 마스크 이미지를 생성한다.
    fn create_mask(&self, hsv_frame: &Mat, threshold: ((u8, u8, u8), (u8, u8, u8))) -> Result<Mat> {
        let mut mask = Mat::default();
        let (lower, upper) = threshold;
        let lower_bound = Scalar::new(lower.0 as f64, lower.1 as f64, lower.2 as f64, 0.0);
        let upper_bound = Scalar::new(upper.0 as f64, upper.1 as f64, upper.2 as f64, 0.0);
        core::in_range(hsv_frame, &lower_bound, &upper_bound, &mut mask)?;
        Ok(mask)
    }

    /// 모폴로지 열림(Opening) 연산을 적용해 마스크 노이즈를 제거한다.
    /// 인자로 전달된 바이너리 마스크를 변환한 `Mat`을 반환한다.
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

    /// 마스크 이미지에서 가장 큰 픽셀 클러스터 크기를 계산한다.
    /// DBSCAN 결과를 기반으로 핵심/경계 포인트를 모두 카운트하며, 클러스터가 없으면 0을 반환한다.
    fn find_largest_cluster(&self, mask: &Mat) -> usize {
        self.find_largest_cluster_with(mask, self.dbscan_epsilon, self.dbscan_min_points)
    }

    /// 다운스케일 등 상황에 맞춰 epsilon/min_points를 외부에서 지정할 수 있는 변형.
    fn find_largest_cluster_with(&self, mask: &Mat, epsilon: f64, min_points: usize) -> usize {
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
        let model = dbscan::Model::new(epsilon, min_points);

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

    /// HSV/BGR 입력을 받아 다운스케일링 후 색상 마스크를 색상 채널로 합성한 디버그 이미지를 생성한다.
    /// 반환 이미지는 BGR 320x240 크기다.
    pub fn debug_visualize_bgr(&self, bgr_input: &Mat) -> Result<Mat> {
        // 입력을 HSV로 변환
        let hsv = {
            let mut hsv = Mat::default();
            imgproc::cvt_color(
                bgr_input,
                &mut hsv,
                imgproc::COLOR_BGR2HSV,
                0,
                ALGO_HINT_DEFAULT,
            )?;
            hsv
        };

        // 다운스케일 (원본이 작으면 그대로 사용)
        let orig_w = hsv.cols().max(1);
        let orig_h = hsv.rows().max(1);
        let target_w = 320;
        let target_h = 240;
        let mut hsv_small = Mat::default();
        let mut bgr_small = Mat::default();
        if orig_w <= target_w || orig_h <= target_h {
            hsv_small = hsv;
            bgr_small = bgr_input.clone();
        } else {
            imgproc::resize(
                &hsv,
                &mut hsv_small,
                Size::new(target_w, target_h),
                0.0,
                0.0,
                imgproc::INTER_AREA,
            )?;
            imgproc::resize(
                bgr_input,
                &mut bgr_small,
                Size::new(target_w, target_h),
                0.0,
                0.0,
                imgproc::INTER_AREA,
            )?;
        }

        // 마스크 생성 + 노이즈 제거
        let red_mask = self.create_mask(&hsv_small, self.red_threshold)?;
        let yellow_mask = self.create_mask(&hsv_small, self.yellow_threshold)?;
        let green_mask = self.create_mask(&hsv_small, self.green_threshold)?;

        let red = self.apply_morphology(&red_mask).unwrap_or(red_mask);
        let yellow = self
            .apply_morphology(&yellow_mask)
            .unwrap_or(yellow_mask);
        let green = self.apply_morphology(&green_mask).unwrap_or(green_mask);

        // 채널 합성: B=0, G=max(green, yellow), R=max(red, yellow)
        let zeros = Mat::zeros(hsv_small.rows(), hsv_small.cols(), red.typ())?.to_mat()?;
        let mut g_chan = Mat::default();
        core::max(&green, &yellow, &mut g_chan)?;
        let mut r_chan = Mat::default();
        core::max(&red, &yellow, &mut r_chan)?;
        let mut bgr = Mat::default();
        let mut mv = Vector::<Mat>::new();
        mv.push(zeros);
        mv.push(g_chan);
        mv.push(r_chan);
        core::merge(&mv, &mut bgr)?;
        // 원본(또는 다운스케일) BGR과 마스크 시각화를 오버레이한다.
        let mut overlay = Mat::default();
        // 안전을 위해 bgr_small가 비어 있으면 bgr를 베이스로 사용
        let base = if bgr_small.empty() { &bgr } else { &bgr_small };
        core::add_weighted(base, 0.6, &bgr, 0.4, 0.0, &mut overlay, -1)?;

        Ok(overlay)
    }
}

/// 캘리브레이션 구조체를 `(하한, 상한)` 튜플로 변환한다.
fn thresholds_to_tuple(threshold: TrafficLightColorThreshold) -> ((u8, u8, u8), (u8, u8, u8)) {
    (threshold.lower, threshold.upper)
}
