// asw/vision

use crate::asw::lib::vs_lane_lib::*;
use crate::rte::rte_dto::{DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed};
use crate::rte::rte_main::CameraChannels;
use opencv::core::Mat;
use opencv::prelude::MatTraitConst;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::RecvError;

const PROCESS_INTERVAL: u32 = 3;

pub async fn runnable_pre_processing(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let processed_tx = camera.processed_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;

        let lane_config = LaneTaskConfig::new(false);
        let pipeline = Pipeline::new_with_settings(lane_config.use_kalman)?;
        let mut frame_counter: u32 = 0;
        let mut fps_start = Instant::now();
        let mut last_processed_frame: Option<Arc<Mat>> = None;

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

            let should_process =
                (alive_cnt % PROCESS_INTERVAL == 0) || last_processed_frame.is_none();

            let processed_arc: Arc<Mat> = if should_process {
                let bgr_mat = cam_raw.as_bgr_mat()?;
                let gray = pipeline.gray_scale(&bgr_mat)?;
                let blur = pipeline.noise_removal(&gray)?;
                let edges = pipeline.edge_detection(&blur)?;
                let closed = pipeline.morphology_close(&edges)?;
                let new_arc = Arc::new(closed);
                last_processed_frame = Some(new_arc.clone());
                new_arc
            } else {
                // Safety: last_processed_frame is always populated if we skip processing.
                last_processed_frame
                    .as_ref()
                    .expect("processed frame cache missing")
                    .clone()
            };

            // 3. 결과 전송
            let processed_width = processed_arc.cols() as u32;
            let processed_height = processed_arc.rows() as u32;
            let preprocessed_dto = Arc::new(DtoCamProcessed::new(
                processed_arc.clone(),
                processed_width,
                processed_height,
                alive_cnt,
            ));
            let _ = processed_tx.send(preprocessed_dto);

            alive_cnt += 1;
            frame_counter += 1;

            let elapsed = fps_start.elapsed();
            if elapsed >= Duration::from_secs(1) {
                let fps = frame_counter as f64 / elapsed.as_secs_f64();
                println!("[{}] PreProcess FPS: {:.2}", id, fps);
                frame_counter = 0;
                fps_start = Instant::now();
            }
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

        let lane_config = LaneTaskConfig::new(false);
        let mut pipeline = Pipeline::new_with_settings(lane_config.use_kalman)?;
        if lane_config.use_kalman {
            pipeline.reset_kalman(0.0, 1.0);
        }
        let mut frame_counter: u32 = 0;
        let mut fps_start = Instant::now();
        let mut last_birds_eye: Option<Arc<Mat>> = None;
        let mut last_angle: f64 = 0.0;
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

            let should_process = (alive_cnt % PROCESS_INTERVAL == 0) || last_birds_eye.is_none();

            let (birds_eye_arc, steering_angle) = if should_process {
                let roi_img = pipeline.roi(&cam_processed.img)?;
                let birds_eye_img = pipeline.perspective_transform(&roi_img)?;
                let (_debug, left_fitx, right_fitx, left_detected, right_detected) =
                    pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;

                if !(left_detected || right_detected) {
                    pipeline.left_a.clear();
                    pipeline.left_b.clear();
                    pipeline.left_c.clear();
                    pipeline.right_a.clear();
                    pipeline.right_b.clear();
                    pipeline.right_c.clear();
                }

                let detected_angle = pipeline.get_angle_on_lane(
                    &left_fitx,
                    &right_fitx,
                    left_detected,
                    right_detected,
                );

                let steering_angle = if lane_config.use_kalman {
                    pipeline.update_angle_kalman(detected_angle)
                } else {
                    detected_angle
                };

                let birds_eye_arc = Arc::new(birds_eye_img);
                last_birds_eye = Some(birds_eye_arc.clone());
                (birds_eye_arc, steering_angle)
            } else {
                let birds_eye_arc = last_birds_eye
                    .as_ref()
                    .expect("birds-eye cache missing")
                    .clone();
                let steering_angle = last_angle;
                (birds_eye_arc, steering_angle)
            };
            last_angle = steering_angle;

            // 3. 결과 전송
            let lane_angle_dto = Arc::new(DtoCamLaneAngle::new(steering_angle, alive_cnt));
            let _ = lane_angle_tx.send(lane_angle_dto);

            let birds_eye_view_dto = Arc::new(DtoCamBirdEyeView::new(
                birds_eye_arc.clone(),
                birds_eye_arc.cols() as u32,
                birds_eye_arc.rows() as u32,
                alive_cnt,
            ));
            let _ = bird_eye_tx.send(birds_eye_view_dto);

            alive_cnt += 1;
            frame_counter += 1;

            let elapsed = fps_start.elapsed();
            if elapsed >= Duration::from_secs(1) {
                let fps = frame_counter as f64 / elapsed.as_secs_f64();
                println!("[{}] LaneAngle FPS: {:.2}", id, fps);
                frame_counter = 0;
                fps_start = Instant::now();
            }
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
