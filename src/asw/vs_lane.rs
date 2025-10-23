//! Lane processing tasks: pre-processing raw frames and estimating the lane angle.
//! The heavy lifting lives in `asw::lib::vs_lane_lib::Pipeline`; this module
//! wires it up to the RTE channels and applies the configurable calibration set.

// asw/vision

use crate::asw::lib::vs_lane_lib::*;
use crate::calibration::{LaneCalibration, LaneCalibrationPreset};
use crate::rte::rte_dto::{ColorFormat, DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed};
use crate::rte::rte_main::CameraChannels;
use opencv::core::Mat;
use opencv::prelude::MatTraitConst;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast::error::RecvError, oneshot};

/// Consume raw camera frames, apply the image pre-processing pipeline and
/// publish the processed grayscale output for downstream stages.
pub async fn runnable_pre_processing(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let processed_tx = camera.processed_tx.clone();
    let (done_tx, done_rx) = oneshot::channel();
    let calibration = LaneCalibration::preset(LaneCalibrationPreset::Vga640x480);
    let runtime_calibration = calibration.runtime;
    let lane_task_config = LaneTaskConfig::new(calibration.processing.kalman.enabled);

    thread::Builder::new()
        .name(format!("lane-preprocess-{}", id))
        .spawn(move || {
            let mut rx = raw_tx.subscribe();
            let mut alive_cnt = 0;

            let pipeline = match Pipeline::new_with_settings(lane_task_config.use_kalman) {
                Ok(pipeline) => pipeline,
                Err(err) => {
                    let _ = done_tx.send(Err(err));
                    return;
                }
            };
            let process_interval = runtime_calibration.process_interval;
            let mut frame_counter: u32 = 0;
            let mut fps_start = Instant::now();
            let mut last_processed_frame: Option<Arc<Mat>> = None;

            let result = (|| -> opencv::Result<()> {
                loop {
                    let mut cam_raw = match rx.blocking_recv() {
                        Ok(cam_dto) => cam_dto,
                        Err(RecvError::Lagged(n)) => {
                            eprintln!("[{}] PreProcess lagged by {}", id, n);
                            continue;
                        }
                        Err(RecvError::Closed) => break,
                    };

                    while let Ok(newer) = rx.try_recv() {
                        cam_raw = newer;
                    }
                    // allow calibration to throttle expensive work or keep going every frame
                    let should_process = process_interval == 0
                        || (alive_cnt % process_interval == 0)
                        || last_processed_frame.is_none();

                    let processed_arc: Arc<Mat> = if should_process {
                        let initial_gray = if matches!(cam_raw.color_format, ColorFormat::Gray) {
                            let gray_view = cam_raw.as_mat_view()?;
                            pipeline.mat_to_umat(&gray_view)?
                        } else {
                            let bgr_mat = cam_raw.as_bgr_mat()?;
                            pipeline.gray_scale(&bgr_mat)?
                        };
                        let blur = pipeline.noise_removal(&initial_gray)?;
                        let edges = pipeline.edge_detection(&blur)?;
                        let closed = pipeline.morphology_close(&edges)?;
                        let closed_mat = pipeline.umat_to_mat(&closed)?;
                        let new_arc = Arc::new(closed_mat);
                        last_processed_frame = Some(new_arc.clone());
                        new_arc
                    } else {
                        last_processed_frame
                            .as_ref()
                            .expect("processed frame cache missing")
                            .clone()
                    };

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
            })();

            let _ = done_tx.send(result);
        })
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Failed to spawn lane pre-processing thread: {}", e),
            )
        })?;

    match done_rx.await {
        Ok(res) => res,
        Err(_) => Err(opencv::Error::new(
            opencv::core::StsError,
            "Lane pre-processing thread ended unexpectedly",
        )),
    }
}

/// Consume pre-processed frames, compute the bird's-eye projection and lane
/// angle, and publish both the visualization and the numeric steering value.
pub async fn runnable_get_lane_angle(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let processed_tx = camera.processed_tx.clone();
    let bird_eye_tx = camera.bird_eye_tx.clone();
    let lane_angle_tx = camera.lane_angle_tx.clone();
    let (done_tx, done_rx) = oneshot::channel();
    let calibration = LaneCalibration::preset(LaneCalibrationPreset::Vga640x480);
    let runtime_calibration = calibration.runtime;
    let lane_task_config = LaneTaskConfig::new(calibration.processing.kalman.enabled);

    thread::Builder::new()
        .name(format!("lane-angle-{}", id))
        .spawn(move || {
            let mut rx = processed_tx.subscribe();
            let mut alive_cnt: u32 = 0;

            let mut pipeline = match Pipeline::new_with_settings(lane_task_config.use_kalman) {
                Ok(p) => p,
                Err(err) => {
                    let _ = done_tx.send(Err(err));
                    return;
                }
            };
            if lane_task_config.use_kalman {
                let kalman = calibration.processing.kalman;
                pipeline.reset_kalman(kalman.initial_estimate, kalman.initial_covariance);
            }
            let process_interval = runtime_calibration.process_interval;
            let mut frame_counter: u32 = 0;
            let mut fps_start = Instant::now();
            let mut last_birds_eye: Option<Arc<Mat>> = None;
            let mut last_angle: f64 = 0.0;

            let result = (|| -> opencv::Result<()> {
                loop {
                    let mut cam_processed = match rx.blocking_recv() {
                        Ok(cam_dto) => cam_dto,
                        Err(RecvError::Lagged(n)) => {
                            eprintln!("[{}] LaneAngle lagged by {}", id, n);
                            continue;
                        }
                        Err(RecvError::Closed) => break,
                    };

                    while let Ok(newer) = rx.try_recv() {
                        cam_processed = newer;
                    }
                    // reuse cached results when throttled by the calibration interval
                    let should_process = process_interval == 0
                        || (alive_cnt % process_interval == 0)
                        || last_birds_eye.is_none();

                    let (birds_eye_arc, steering_angle) = if should_process {
                        let roi_img = pipeline.roi(&cam_processed.img)?;
                        let birds_eye_binary = pipeline.perspective_transform(&roi_img)?;
                        let sliding = calibration.processing.sliding;
                        let (
                            birds_eye_visual,
                            left_fitx,
                            right_fitx,
                            left_detected,
                            right_detected,
                        ) = pipeline.sliding_window(
                            &birds_eye_binary,
                            sliding.window_count,
                            sliding.search_margin,
                            sliding.minpix,
                            sliding.draw_debug_windows,
                        )?;

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

                        let steering_angle = if lane_task_config.use_kalman {
                            pipeline.update_angle_kalman(detected_angle)
                        } else {
                            detected_angle
                        };

                        let birds_eye_arc = Arc::new(birds_eye_visual);
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
            })();

            let _ = done_tx.send(result);
        })
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Failed to spawn lane angle thread: {}", e),
            )
        })?;

    match done_rx.await {
        Ok(res) => res,
        Err(_) => Err(opencv::Error::new(
            opencv::core::StsError,
            "Lane angle thread ended unexpectedly",
        )),
    }
}
