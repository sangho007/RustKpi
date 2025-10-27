//! 차선 추출 비전 태스크 모음.
//! - `asw::lib::vs_lane_lib::Pipeline`에서 제공하는 영상 처리 파이프라인을 연결한다.
//! - RTE 채널과 캘리브레이션 설정을 묶어 전처리/차선 각도 추정 태스크를 구성한다.

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

/// 카메라 RAW 프레임을 받아 전처리 파이프라인을 수행한다.
/// - 색상 변환, 노이즈 제거, 에지 추출, 모폴로지 연산을 순차적으로 적용한다.
/// - 결과를 회색조 `DtoCamProcessed`로 만들어 후속 단계에서 재사용하도록 브로드캐스트한다.
pub async fn runnable_vs_preprocessing(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let processed_tx = camera.processed_tx.clone();
    let (done_tx, done_rx) = oneshot::channel();
    let calibration = LaneCalibration::preset(LaneCalibrationPreset::Vga640x480);
    let runtime_calibration = calibration.runtime;
    let lane_task_config = LaneTaskConfig::new(calibration.processing.kalman.enabled);

    // 영상 처리는 OpenCV `Mat`을 많이 다루므로 전용 OS 스레드에서 수행한다.
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
                    // 캘리브레이션 설정에 따라 프레임을 건너뛰어 CPU 부하를 줄인다.
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
                        // 영상 전처리 파이프라인 순서: 노이즈 제거 → 에지 추출 → 모폴로지 폐연산.
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

/// 전처리된 영상을 받아 버드아이 뷰와 차선 각도를 계산한다.
/// - ROI 추출, 투시 변환, 슬라이딩 윈도우 기반 차선 탐색을 수행한다.
/// - 결과로 생성된 버드아이 시각화와 각도 값을 각각의 RTE 채널에 퍼블리시한다.
pub async fn runnable_vs_get_lane_angle(
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

    // 차선 각도 계산 역시 CPU 집약적이므로 전용 스레드에서 처리한다.
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
                // 칼만 필터를 초기화해 연속 프레임 간 노이즈를 줄인다.
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
                    // 처리 주기가 제한된 경우 이전 결과를 재사용한다.
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
                            // 차선을 찾지 못하면 누적 회귀 계수를 초기화한다.
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

                    // 차선 각도와 버드아이 영상 결과를 각각 전송한다.
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
