//! 메인 런타임: RTE 채널에서 데이터를 수집해 프리뷰 GUI로 전달한다.
//! 또한 IMU/초음파 로그를 출력하고 종료 시그널을 관리한다.

use crate::calibration::LOCALIZATION_ACTIVE_SCENARIO;
use crate::rte::rte_dto::*;
use crate::rte::rte_main::RteChannels;
use crate::util::preview_runtime::{self, FramePacket, FramePayload, PreviewEvent, PreviewMessage};
use opencv::core::{CV_8UC3, Mat, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::{MatExprTraitConst, MatTraitConst, MatTraitConstManual};
use serde::Deserialize;
use std::fs;
use std::sync::{Arc, mpsc};
use std::time::{Duration, Instant};
use tokio::{select, sync::broadcast::error::RecvError};

/// GUI 프리뷰를 활성화할지 여부.
const DEBUG_ON: bool = true;
/// 터미널 로그 출력을 활성화할지 여부.
const TERMINAL_OUTPUT_ON: bool = false;

macro_rules! runtime_println {
    ($($arg:tt)*) => {
        if TERMINAL_OUTPUT_ON {
            ::std::println!($($arg)*);
        }
    };
}

macro_rules! runtime_eprintln {
    ($($arg:tt)*) => {
        if TERMINAL_OUTPUT_ON {
            ::std::eprintln!($($arg)*);
        }
    };
}
const PATH_PREVIEW_INTERVAL: Duration = Duration::from_millis(200);
const LANE_LOG_INTERVAL: Duration = Duration::from_millis(500);
const PATH_CANVAS_SIZE: i32 = 640;

struct MapWaypoints {
    inner: Vec<[f64; 2]>,
    outer: Vec<[f64; 2]>,
}

#[derive(Deserialize)]
struct RawWaypoint {
    position: [f32; 2],
}

#[derive(Deserialize)]
struct RawMap {
    #[serde(default)]
    inner_waypoint: Vec<RawWaypoint>,
    #[serde(default)]
    outer_waypoint: Vec<RawWaypoint>,
}

/// RTE 채널을 사용하며 프리뷰 GUI와 데이터 스트림을 조율하는 메인 런타임 루프를 수행한다.
pub async fn run(channels: RteChannels) -> opencv::Result<()> {
    // 카메라·초음파 채널을 복제해 비동기 작업에서 공유한다.
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();
    let imu_channels = channels.imu.clone();
    let localization_channels = channels.localization.clone();

    let map_waypoints = load_map_waypoints(LOCALIZATION_ACTIVE_SCENARIO.map);

    // 사용자에게 실행 상태를 안내한다.
    runtime_println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

    // 디버그 모드에서는 프리뷰 스레드를 띄워 GUI를 활성화한다.
    let (mut preview_tx, mut preview_event_rx, preview_handle) = if DEBUG_ON {
        let runtime = preview_runtime::spawn_preview_thread()?;
        (
            Some(runtime.tx),
            Some(runtime.event_rx),
            Some(runtime.handle),
        )
    } else {
        (None, None, None)
    };

    // 각 데이터 스트림을 구독한다.
    let mut camraw_rx = camera_channels.raw_tx.subscribe();
    let mut processed_rx = camera_channels.processed_tx.subscribe();
    let mut birds_eye_rx = camera_channels.bird_eye_tx.subscribe();
    let mut lane_angle_rx = camera_channels.lane_angle_tx.subscribe();
    let mut distance_rx = ultrasonic_channels.raw_tx.subscribe();
    let mut imu_rx = imu_channels.parsed_tx.subscribe();
    let mut arrival_rx = localization_channels.arrival_tx.subscribe();
    let mut loc_state_rx = localization_channels.state_tx.subscribe();
    let mut global_path_rx = channels.path.global_tx.subscribe();
    let mut local_path_rx = channels.path.local_tx.subscribe();
    let mut smoothed_path_rx = channels.path.smoothed_tx.subscribe();

    let mut latest_global_path: Option<Arc<DtoAdasGlobalPath>> = None;
    let mut latest_local_path: Option<Arc<DtoAdasLocalPath>> = None;
    let mut latest_smoothed_path: Option<Arc<DtoAdasSmoothedPath>> = None;
    let mut latest_localization_state: Option<DtoLocalizationState> = None;
    let mut last_path_preview = Instant::now()
        .checked_sub(PATH_PREVIEW_INTERVAL)
        .unwrap_or_else(Instant::now);
    let mut last_lane_log = Instant::now()
        .checked_sub(LANE_LOG_INTERVAL)
        .unwrap_or_else(Instant::now);

    // Ctrl-C 입력을 감시해 사용자의 종료 요청을 처리한다.
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    // GUI와 ASW 데이터를 중계하는 메인 이벤트 루프다.
    'main_loop: loop {
        select! {
            biased;

            // 사용자 Ctrl-C 입력을 감지한다. (우선순위를 높여 즉시 종료한다.)
            result = &mut ctrl_c => {
                if let Err(err) = result {
                    runtime_eprintln!("[MAIN] Failed to receive Ctrl-C signal: {}", err);
                } else {
                    runtime_println!("[MAIN] Ctrl-C received, shutting down...");
                }
                break 'main_loop;
            },

            // 최신 원시 카메라 프레임을 프리뷰로 전달한다.
            result = camraw_rx.recv() => match result {
                Ok(camraw) => {
                    let mut newest = camraw;
                    while let Ok(newer) = camraw_rx.try_recv() {
                        newest = newer;
                    }

                    if let Some(tx) = preview_tx.as_ref() {
                        let payload = FramePacket {
                            width: newest.width,
                            height: newest.height,
                            stride: newest.stride,
                            format: newest.color_format,
                            payload: FramePayload::Camera(newest.buffer.clone()),
                        };
                        let _ = tx.send(PreviewMessage::Raw(payload));
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] raw frame lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] raw frame channel closed.");
                    break 'main_loop;
                }
            },

            // 전처리된 프레임을 프리뷰에 갱신한다.
            result = processed_rx.recv() => match result {
                Ok(cam_processed) => {
                    let mut newest = cam_processed;
                    while let Ok(newer) = processed_rx.try_recv() {
                        newest = newer;
                    }

                    if let Some(tx) = preview_tx.as_ref() {
                        let mat = newest.img.clone();
                        match mat.as_ref().step1(0) {
                            Ok(stride) => {
                                let format = mat_color_format(mat.as_ref());
                                let payload = FramePacket {
                                    width: newest.width,
                                    height: newest.height,
                                    stride: stride as usize,
                                    format,
                                    payload: FramePayload::Mat(mat),
                                };
                                let _ = tx.send(PreviewMessage::Processed(payload));
                            }
                            Err(err) => {
                                runtime_eprintln!("[GUI] Failed to read processed stride: {}", err);
                            }
                        }
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Processed frame lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] Processed frame channel closed.");
                    break 'main_loop;
                }
            },

            // 버드아이 뷰 프레임을 갱신한다.
            result = birds_eye_rx.recv() => match result {
                Ok(birds_eye) => {
                    let mut newest = birds_eye;
                    while let Ok(newer) = birds_eye_rx.try_recv() {
                        newest = newer;
                    }

                    if let Some(tx) = preview_tx.as_ref() {
                        let mat = newest.img.clone();
                        match mat.as_ref().step1(0) {
                            Ok(stride) => {
                                let format = mat_color_format(mat.as_ref());
                                let payload = FramePacket {
                                    width: newest.width,
                                    height: newest.height,
                                    stride: stride as usize,
                                    format,
                                    payload: FramePayload::Mat(mat),
                                };
                                let _ = tx.send(PreviewMessage::Bird(payload));
                            }
                            Err(err) => {
                                runtime_eprintln!("[GUI] Failed to read bird-eye stride: {}", err);
                            }
                        }
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Bird eye stream lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] Bird eye channel closed.");
                    break 'main_loop;
                }
            },
            // 전역 경로 업데이트를 처리한다.
            result = global_path_rx.recv() => match result {
                Ok(path_arc) => {
                    let mut newest = path_arc;
                    while let Ok(newer) = global_path_rx.try_recv() {
                        newest = newer;
                    }
                    latest_global_path = Some(newest);
                    maybe_publish_path_preview(
                        preview_tx.as_ref(),
                        latest_global_path.as_ref(),
                        latest_local_path.as_ref(),
                        latest_smoothed_path.as_ref(),
                        latest_localization_state.as_ref(),
                        map_waypoints.as_ref(),
                        &mut last_path_preview,
                    );
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Global path lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_println!("[MAIN] Global path channel closed, shutting down...");
                    break 'main_loop;
                }
            },
            // 로컬 경로 업데이트를 처리한다.
            result = local_path_rx.recv() => match result {
                Ok(path_arc) => {
                    let mut newest = path_arc;
                    while let Ok(newer) = local_path_rx.try_recv() {
                        newest = newer;
                    }
                    latest_local_path = Some(newest);
                    maybe_publish_path_preview(
                        preview_tx.as_ref(),
                        latest_global_path.as_ref(),
                        latest_local_path.as_ref(),
                        latest_smoothed_path.as_ref(),
                        latest_localization_state.as_ref(),
                        map_waypoints.as_ref(),
                        &mut last_path_preview,
                    );
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Local path lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_println!("[MAIN] Local path channel closed, shutting down...");
                    break 'main_loop;
                }
            },
            // 스무딩 경로 업데이트를 처리한다.
            result = smoothed_path_rx.recv() => match result {
                Ok(path_arc) => {
                    let mut newest = path_arc;
                    while let Ok(newer) = smoothed_path_rx.try_recv() {
                        newest = newer;
                    }
                    latest_smoothed_path = Some(newest);
                    maybe_publish_path_preview(
                        preview_tx.as_ref(),
                        latest_global_path.as_ref(),
                        latest_local_path.as_ref(),
                        latest_smoothed_path.as_ref(),
                        latest_localization_state.as_ref(),
                        map_waypoints.as_ref(),
                        &mut last_path_preview,
                    );
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Smoothed path lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_println!("[MAIN] Smoothed path channel closed, shutting down...");
                    break 'main_loop;
                }
            },

            // 차선 각도 결과를 로그로 출력한다.
            result = lane_angle_rx.recv() => match result {
                Ok(lane_angle) => {
                    if last_lane_log.elapsed() >= LANE_LOG_INTERVAL {
                        runtime_println!(
                            "[LANE] angle={:.2}deg offset={:.2}px alive_cnt={}",
                            lane_angle.angle,
                            lane_angle.lateral_offset,
                            lane_angle.alive_cnt
                        );
                        last_lane_log = Instant::now();
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Lane angle lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] Lane angle channel closed.");
                    break 'main_loop;
                }
            },

            // 초음파 거리 정보를 출력한다.
            result = distance_rx.recv() => match result {
                Ok(distance) => {
                    runtime_println!("distance: {}, alive_cnt: {}", distance.distance, distance.alive_cnt);
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Uss lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] Uss angle channel closed.");
                    break 'main_loop;
                }
            },

            // 목적지 도착 여부를 감시한다.
            result = arrival_rx.recv() => match result {
                Ok(dto) => {
                    let mut latest = dto;
                    while let Ok(newer) = arrival_rx.try_recv() {
                        latest = newer;
                    }
                    if latest.arrived {
                        runtime_println!(
                            "[MAIN] Destination reached (timestamp_ns={}), shutting down...",
                            latest.timestamp_ns
                        );
                        break 'main_loop;
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Localization arrival lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_println!("[MAIN] Localization arrival channel closed, shutting down...");
                    break 'main_loop;
                }
            },

            // Localization 상태를 갱신해 경로 시각화에 반영한다.
            result = loc_state_rx.recv() => match result {
                Ok(state_arc) => {
                    let mut newest = state_arc;
                    while let Ok(newer) = loc_state_rx.try_recv() {
                        newest = newer;
                    }
                    latest_localization_state = Some(newest.as_ref().clone());
                    maybe_publish_path_preview(
                        preview_tx.as_ref(),
                        latest_global_path.as_ref(),
                        latest_local_path.as_ref(),
                        latest_smoothed_path.as_ref(),
                        latest_localization_state.as_ref(),
                        map_waypoints.as_ref(),
                        &mut last_path_preview,
                    );
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] Localization state lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_println!("[MAIN] Localization state channel closed, shutting down...");
                    break 'main_loop;
                }
            },
            // IMU DTO를 출력해 데이터 흐름을 확인한다.
            result = imu_rx.recv() => match result {
                Ok(imu) => {
                    let header = &imu.header;
                    let pose_position = imu
                        .pose
                        .as_ref()
                        .and_then(|pose| pose.position_world);
                    let gyro_body = imu
                        .gyro
                        .as_ref()
                        .and_then(|gyro| gyro.body);
                    runtime_println!(
                        "[IMU] header={{stamp_ns={}, dt_ns={}, seq={}, session_id={:?}, clock_domain={:?}, frame_id={:?}, child_frame_id={:?}}} alive_cnt={} position_world={:?} gyro_body={:?}",
                        header.stamp_ns,
                        header.dt_ns,
                        header.seq,
                        header.session_id,
                        header.clock_domain,
                        header.frame_id,
                        header.child_frame_id,
                        imu.alive_cnt,
                        pose_position,
                        gyro_body
                    );
                }
                Err(RecvError::Lagged(n)) => {
                    runtime_eprintln!("[MAIN] IMU stream lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    runtime_eprintln!("[MAIN] IMU channel closed.");
                    break 'main_loop;
                }
            },
        }

        // GUI에서 종료 이벤트를 요청하면 즉시 빠져나간다.
        if let Some(event_rx) = preview_event_rx.as_mut() {
            if let Ok(PreviewEvent::Quit) = event_rx.try_recv() {
                runtime_println!("[MAIN] Preview requested quit, shutting down...");
                break 'main_loop;
            }
        }
    }

    // 프리뷰 송신자를 정리한다.
    if let Some(tx) = preview_tx.take() {
        drop(tx);
    }

    // 프리뷰 스레드를 종료까지 대기한다.
    if let Some(handle) = preview_handle {
        if let Err(err) = handle.join() {
            runtime_eprintln!("[MAIN] Failed to join preview thread: {:?}", err);
        }
    }

    runtime_println!("== 시뮬레이션 종료 ==");
    Ok(())
}

fn maybe_publish_path_preview(
    preview_tx: Option<&mpsc::Sender<PreviewMessage>>,
    global_path: Option<&Arc<DtoAdasGlobalPath>>,
    local_path: Option<&Arc<DtoAdasLocalPath>>,
    smoothed_path: Option<&Arc<DtoAdasSmoothedPath>>,
    localization_state: Option<&DtoLocalizationState>,
    map_waypoints: Option<&MapWaypoints>,
    last_sent: &mut Instant,
) {
    if preview_tx.is_none() {
        return;
    }
    if last_sent.elapsed() < PATH_PREVIEW_INTERVAL {
        return;
    }
    let frame = match build_path_preview_frame(
        global_path,
        local_path,
        smoothed_path,
        localization_state,
        map_waypoints,
    ) {
        Some(frame) => frame,
        None => return,
    };
    if let Some(tx) = preview_tx {
        if tx.send(PreviewMessage::Path(frame)).is_ok() {
            *last_sent = Instant::now();
        }
    }
}

fn build_path_preview_frame(
    global_path: Option<&Arc<DtoAdasGlobalPath>>,
    local_path: Option<&Arc<DtoAdasLocalPath>>,
    smoothed_path: Option<&Arc<DtoAdasSmoothedPath>>,
    localization_state: Option<&DtoLocalizationState>,
    map_waypoints: Option<&MapWaypoints>,
) -> Option<FramePacket> {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut has_points = false;

    if let Some(map) = map_waypoints {
        for pos in &map.inner {
            let x = pos[0];
            let y = pos[1];
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            has_points = true;
        }
        for pos in &map.outer {
            let x = pos[0];
            let y = pos[1];
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            has_points = true;
        }
    }

    if let Some(path) = global_path {
        for wp in &path.waypoints {
            let x = wp.position_xy[0] as f64;
            let y = wp.position_xy[1] as f64;
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            has_points = true;
        }
    }
    if let Some(path) = local_path {
        for wp in &path.waypoints {
            let x = wp.position_xy[0] as f64;
            let y = wp.position_xy[1] as f64;
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            has_points = true;
        }
    }
    if let Some(path) = smoothed_path {
        for sample in &path.samples_xy {
            let x = sample[0] as f64;
            let y = sample[1] as f64;
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            has_points = true;
        }
    }
    if let Some(state) = localization_state {
        let x = state.position_map_xy[0];
        let y = state.position_map_xy[1];
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
        has_points = true;
    }

    if !has_points {
        return None;
    }

    if (max_x - min_x).abs() < 1e-3 {
        min_x -= 1.0;
        max_x += 1.0;
    }
    if (max_y - min_y).abs() < 1e-3 {
        min_y -= 1.0;
        max_y += 1.0;
    }

    let span_x = max_x - min_x;
    let span_y = max_y - min_y;
    let pad_x = span_x * 0.08 + 1.0;
    let pad_y = span_y * 0.08 + 1.0;
    min_x -= pad_x;
    max_x += pad_x;
    min_y -= pad_y;
    max_y += pad_y;

    let span_x = (max_x - min_x).max(f64::EPSILON);
    let span_y = (max_y - min_y).max(f64::EPSILON);
    let draw_width = (PATH_CANVAS_SIZE as f64) - 40.0;
    let draw_height = (PATH_CANVAS_SIZE as f64) - 40.0;
    let scale_x = draw_width / span_x;
    let scale_y = draw_height / span_y;
    let scale = scale_x.min(scale_y).max(f64::MIN_POSITIVE);

    let actual_width = span_x * scale;
    let actual_height = span_y * scale;
    let offset_x = 20.0 + (draw_width - actual_width) / 2.0;
    let offset_y = 20.0 + (draw_height - actual_height) / 2.0;

    let to_point = |x: f64, y: f64| -> Point {
        let mut px = ((x - min_x) * scale + offset_x).round() as i32;
        let mut py = (((max_y - y) * scale) + offset_y).round() as i32;
        let max_coord = PATH_CANVAS_SIZE - 1;
        px = px.clamp(0, max_coord);
        py = py.clamp(0, max_coord);
        Point::new(px, py)
    };

    let expr = match Mat::zeros(PATH_CANVAS_SIZE, PATH_CANVAS_SIZE, CV_8UC3) {
        Ok(expr) => expr,
        Err(_) => return None,
    };
    let mut canvas = match expr.to_mat() {
        Ok(mat) => mat,
        Err(_) => return None,
    };

    if let Some(map) = map_waypoints {
        let mut previous: Option<Point> = None;
        for pos in &map.inner {
            let pt = to_point(pos[0], pos[1]);
            if let Some(prev) = previous {
                let _ = imgproc::line(
                    &mut canvas,
                    prev,
                    pt,
                    Scalar::new(70.0, 70.0, 70.0, 0.0),
                    1,
                    imgproc::LINE_AA,
                    0,
                );
            }
            let _ = imgproc::circle(
                &mut canvas,
                pt,
                2,
                Scalar::new(110.0, 110.0, 110.0, 0.0),
                -1,
                imgproc::LINE_AA,
                0,
            );
            previous = Some(pt);
        }

        let mut previous_outer: Option<Point> = None;
        for pos in &map.outer {
            let pt = to_point(pos[0], pos[1]);
            if let Some(prev) = previous_outer {
                let _ = imgproc::line(
                    &mut canvas,
                    prev,
                    pt,
                    Scalar::new(110.0, 110.0, 110.0, 0.0),
                    1,
                    imgproc::LINE_AA,
                    0,
                );
            }
            let _ = imgproc::circle(
                &mut canvas,
                pt,
                2,
                Scalar::new(170.0, 170.0, 170.0, 0.0),
                -1,
                imgproc::LINE_AA,
                0,
            );
            previous_outer = Some(pt);
        }
    }

    if let Some(path) = global_path {
        let mut previous: Option<Point> = None;
        for wp in &path.waypoints {
            let pt = to_point(wp.position_xy[0] as f64, wp.position_xy[1] as f64);
            if let Some(prev) = previous {
                let _ = imgproc::line(
                    &mut canvas,
                    prev,
                    pt,
                    Scalar::new(128.0, 128.0, 255.0, 0.0),
                    2,
                    imgproc::LINE_AA,
                    0,
                );
            }
            previous = Some(pt);
        }
    }

    if let Some(path) = local_path {
        let mut previous: Option<Point> = None;
        for wp in &path.waypoints {
            let pt = to_point(wp.position_xy[0] as f64, wp.position_xy[1] as f64);
            if let Some(prev) = previous {
                let _ = imgproc::line(
                    &mut canvas,
                    prev,
                    pt,
                    Scalar::new(80.0, 255.0, 80.0, 0.0),
                    3,
                    imgproc::LINE_AA,
                    0,
                );
            }
            let _ = imgproc::circle(
                &mut canvas,
                pt,
                3,
                Scalar::new(120.0, 255.0, 120.0, 0.0),
                -1,
                imgproc::LINE_AA,
                0,
            );
            previous = Some(pt);
        }
    }

    if let Some(path) = smoothed_path {
        let mut previous: Option<Point> = None;
        for sample in &path.samples_xy {
            let pt = to_point(sample[0] as f64, sample[1] as f64);
            if let Some(prev) = previous {
                let _ = imgproc::line(
                    &mut canvas,
                    prev,
                    pt,
                    Scalar::new(40.0, 200.0, 255.0, 0.0),
                    2,
                    imgproc::LINE_AA,
                    0,
                );
            }
            previous = Some(pt);
        }
    }

    if let Some(state) = localization_state {
        let pos = to_point(state.position_map_xy[0], state.position_map_xy[1]);
        let heading = state.yaw_rad;
        let heading_len = 1.5;
        let hx = state.position_map_xy[0] + heading_len * heading.cos();
        let hy = state.position_map_xy[1] + heading_len * heading.sin();
        let head_pt = to_point(hx, hy);
        let _ = imgproc::circle(
            &mut canvas,
            pos,
            6,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            -1,
            imgproc::LINE_AA,
            0,
        );
        let _ = imgproc::line(
            &mut canvas,
            pos,
            head_pt,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            2,
            imgproc::LINE_AA,
            0,
        );
    }

    let label_position = Point::new(18, 28);
    let _ = imgproc::put_text(
        &mut canvas,
        "Inner Lane",
        Point::new(label_position.x, label_position.y),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(70.0, 70.0, 70.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );
    let _ = imgproc::put_text(
        &mut canvas,
        "Outer Lane",
        Point::new(label_position.x, label_position.y + 22),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(110.0, 110.0, 110.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );
    let _ = imgproc::put_text(
        &mut canvas,
        "Global Path",
        Point::new(label_position.x, label_position.y + 44),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(128.0, 128.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );
    let _ = imgproc::put_text(
        &mut canvas,
        "Local Path",
        Point::new(label_position.x, label_position.y + 66),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(80.0, 255.0, 80.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );
    let _ = imgproc::put_text(
        &mut canvas,
        "Smoothed Path",
        Point::new(label_position.x, label_position.y + 88),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(40.0, 200.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );
    let _ = imgproc::put_text(
        &mut canvas,
        "Pose",
        Point::new(label_position.x, label_position.y + 110),
        imgproc::FONT_HERSHEY_SIMPLEX,
        0.5,
        Scalar::new(0.0, 0.0, 255.0, 0.0),
        1,
        imgproc::LINE_AA,
        false,
    );

    let data = canvas.data_bytes().ok()?.to_vec();
    Some(FramePacket {
        width: PATH_CANVAS_SIZE as u32,
        height: PATH_CANVAS_SIZE as u32,
        stride: (PATH_CANVAS_SIZE as usize) * 3,
        format: ColorFormat::Bgr,
        payload: FramePayload::Owned(data),
    })
}

fn load_map_waypoints(map_id: crate::calibration::LocalizationMapId) -> Option<MapWaypoints> {
    let path = map_id.json_asset();
    let text = match fs::read_to_string(path) {
        Ok(text) => text,
        Err(err) => {
            runtime_eprintln!("[MAIN] Failed to read map file {}: {}", path, err);
            return None;
        }
    };
    let raw: RawMap = match serde_json::from_str(&text) {
        Ok(raw) => raw,
        Err(err) => {
            runtime_eprintln!("[MAIN] Failed to parse map file {}: {}", path, err);
            return None;
        }
    };

    let RawMap {
        inner_waypoint,
        outer_waypoint,
    } = raw;

    let inner = inner_waypoint
        .into_iter()
        .map(|wp| [wp.position[0] as f64, wp.position[1] as f64])
        .collect::<Vec<_>>();
    let outer = outer_waypoint
        .into_iter()
        .map(|wp| [wp.position[0] as f64, wp.position[1] as f64])
        .collect::<Vec<_>>();

    if inner.is_empty() && outer.is_empty() {
        None
    } else {
        Some(MapWaypoints { inner, outer })
    }
}

/// OpenCV Mat의 채널 수를 기준으로 프리뷰에 사용할 색상 포맷을 결정한다.
fn mat_color_format(mat: &Mat) -> ColorFormat {
    match mat.channels() {
        1 => ColorFormat::Gray,
        3 => ColorFormat::Bgr,
        4 => ColorFormat::Rgba,
        ch => {
            // 지원하지 않는 채널 수는 경고를 남기고 BGR로 폴백한다.
            runtime_eprintln!("[GUI] Unsupported channel count for preview: {}", ch);
            ColorFormat::Bgr
        }
    }
}
