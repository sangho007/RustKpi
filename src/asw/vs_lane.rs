// asw/vision

use crate::asw::lib::vs_lane_lib::*;
pub use crate::asw::lib::vs_lane_lib::LaneDetectionMode;
use crate::rte::rte_dto::{DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed};
use crate::rte::rte_main::CameraChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;



pub async fn runnable_pre_processing(
    id: &'static str,
    camera: CameraChannels,
    
) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let processed_tx = camera.processed_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;

        let lane_config = LaneTaskConfig::new(LaneDetectionMode::Hough, false);
        let pipeline = Pipeline::new_with_settings(lane_config.mode, lane_config.use_kalman)?;

        loop {
            // 1. 이벤트 수신 및 데이터 준비
            let cam_raw = match rx.blocking_recv() {
                Ok(cam_dto) => cam_dto, // 처리할 데이터만 추출
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[{}] PreProcess lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            };

            // 2. 실제 연산 처리
            let gray = pipeline.gray_scale(&cam_raw.img)?;
            let blur = pipeline.noise_removal(&gray)?;
            let edges = pipeline.edge_detection(&blur)?;
            let closed = pipeline.morphology_close(&edges)?;

            // 3. 결과 전송
            let preprocessed_dto = Arc::new(DtoCamProcessed::new(Arc::new(closed), 1280, 720, alive_cnt));
            let _ = processed_tx.send(preprocessed_dto);

            alive_cnt += 1;
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Lane pre-processing task join error: {}", e),
        )
    })?;

    join_result
}

pub async fn runnable_get_lane_angle(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let processed_tx = camera.processed_tx.clone();
    let bird_eye_tx = camera.bird_eye_tx.clone();
    let lane_angle_tx = camera.lane_angle_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = processed_tx.subscribe();
        let mut alive_cnt: u32 = 0;

        let lane_config = LaneTaskConfig::new(LaneDetectionMode::Hough, false);
        let mut pipeline = Pipeline::new_with_settings(lane_config.mode, lane_config.use_kalman)?;
        if lane_config.use_kalman {
            pipeline.reset_kalman(0.0, 1.0);
        }
        loop {
            // 1. 이벤트 수신 및 데이터 준비
            let cam_processed = match rx.blocking_recv() {
                Ok(cam_dto) => cam_dto, // 처리할 데이터만 추출
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[{}] LaneAngle lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            };

            // 2. 실제 연산 처리
            // 3. roi 이미지 추출
            let roi_img = pipeline.roi(&cam_processed.img)?;

            // 4. bird-eye-view 이미지 생성
            let birds_eye_img = pipeline.perspective_transform(&roi_img)?;

            // 5. 차선 검출 모드에 따른 조향각 계산
            let detected_angle = match lane_config.mode {
                LaneDetectionMode::SlidingWindow => {
                    let has_previous_detection = !pipeline.left_a.is_empty() && !pipeline.right_a.is_empty();
                    if has_previous_detection {
                        let (_img, left_fitx, right_fitx, left_detected, right_detected) =
                            pipeline.search_around_poly(&birds_eye_img, 100)?;

                        if !(left_detected || right_detected) {
                            pipeline.left_a.clear();
                            pipeline.left_b.clear();
                            pipeline.left_c.clear();
                            pipeline.right_a.clear();
                            pipeline.right_b.clear();
                            pipeline.right_c.clear();
                        }

                        Some(pipeline.get_angle_on_lane(
                            &left_fitx,
                            &right_fitx,
                            left_detected,
                            right_detected,
                        ))
                    } else {
                        let (_img, left_fitx, right_fitx, left_detected, right_detected) =
                            pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;

                        Some(pipeline.get_angle_on_lane(
                            &left_fitx,
                            &right_fitx,
                            left_detected,
                            right_detected,
                        ))
                    }
                }
                LaneDetectionMode::Hough => {
                    let segments = pipeline.detect_lane_lines_hough(&birds_eye_img)?;
                    pipeline.estimate_angle_from_hough(&segments)
                }
            };

            let steering_angle = if lane_config.use_kalman {
                match detected_angle {
                    Some(angle) => pipeline.update_angle_kalman(angle),
                    None => pipeline.previous_angle(),
                }
            } else {
                detected_angle.unwrap_or_else(|| pipeline.previous_angle())
            };

            // 3. 결과 전송
            let lane_angle_dto = Arc::new(DtoCamLaneAngle::new(steering_angle, alive_cnt));
            let _ = lane_angle_tx.send(lane_angle_dto);

            let birds_eye_view_dto = Arc::new(DtoCamBirdEyeView::new(Arc::new(birds_eye_img), 1280, 720, alive_cnt));
            let _ = bird_eye_tx.send(birds_eye_view_dto);

            alive_cnt += 1;
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Lane angle task join error: {}", e),
        )
    })?;

    join_result
}
