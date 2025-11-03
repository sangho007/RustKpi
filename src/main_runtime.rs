//! 메인 런타임: RTE 채널에서 데이터를 수집해 프리뷰 GUI로 전달한다.
//! 또한 IMU/초음파 로그를 출력하고 종료 시그널을 관리한다.

use crate::rte::rte_dto::*;
use crate::rte::rte_main::RteChannels;
use crate::util::preview_runtime::{self, FramePacket, FramePayload, PreviewEvent, PreviewMessage};
use opencv::core::Mat;
use opencv::prelude::MatTraitConst;
use tokio::{select, sync::broadcast::error::RecvError};

/// GUI 프리뷰를 활성화할지 여부.
const DEBUG_ON: bool = false;

/// RTE 채널을 사용하며 프리뷰 GUI와 데이터 스트림을 조율하는 메인 런타임 루프를 수행한다.
pub async fn run(channels: RteChannels) -> opencv::Result<()> {
    // 카메라·초음파 채널을 복제해 비동기 작업에서 공유한다.
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();
    let imu_channels = channels.imu.clone();
    let localization_channels = channels.localization.clone();

    // 사용자에게 실행 상태를 안내한다.
    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

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

    // Ctrl-C 입력을 감시해 사용자의 종료 요청을 처리한다.
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    // GUI와 ASW 데이터를 중계하는 메인 이벤트 루프다.
    'main_loop: loop {
        select! {
            biased;

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
                    eprintln!("[MAIN] raw frame lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] raw frame channel closed.");
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
                                eprintln!("[GUI] Failed to read processed stride: {}", err);
                            }
                        }
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Processed frame lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Processed frame channel closed.");
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
                                eprintln!("[GUI] Failed to read bird-eye stride: {}", err);
                            }
                        }
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Bird eye stream lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Bird eye channel closed.");
                    break 'main_loop;
                }
            },

            // 차선 각도 결과를 로그로 출력한다.
            result = lane_angle_rx.recv() => match result {
                Ok(lane_angle) => {
                    //println!("Angle: {}, alive_cnt: {}", lane_angle.angle, lane_angle.alive_cnt);
                    ;
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Lane angle lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Lane angle channel closed.");
                    break 'main_loop;
                }
            },

            // 초음파 거리 정보를 출력한다.
            result = distance_rx.recv() => match result {
                Ok(distance) => {
                    println!("distance: {}, alive_cnt: {}", distance.distance, distance.alive_cnt);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Uss lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Uss angle channel closed.");
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
                        println!(
                            "[MAIN] Destination reached (timestamp_ns={}), shutting down...",
                            latest.timestamp_ns
                        );
                        break 'main_loop;
                    }
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Localization arrival lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    println!("[MAIN] Localization arrival channel closed, shutting down...");
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
                    println!(
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
                    eprintln!("[MAIN] IMU stream lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] IMU channel closed.");
                    break 'main_loop;
                }
            },

            // 사용자 Ctrl-C 입력을 감지한다.
            result = &mut ctrl_c => {
                if let Err(err) = result {
                    eprintln!("[MAIN] Failed to receive Ctrl-C signal: {}", err);
                } else {
                    println!("[MAIN] Ctrl-C received, shutting down...");
                }
                break 'main_loop;
            },
        }

        // GUI에서 종료 이벤트를 요청하면 즉시 빠져나간다.
        if let Some(event_rx) = preview_event_rx.as_mut() {
            if let Ok(PreviewEvent::Quit) = event_rx.try_recv() {
                println!("[MAIN] Preview requested quit, shutting down...");
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
            eprintln!("[MAIN] Failed to join preview thread: {:?}", err);
        }
    }

    println!("== 시뮬레이션 종료 ==");
    Ok(())
}

/// OpenCV Mat의 채널 수를 기준으로 프리뷰에 사용할 색상 포맷을 결정한다.
fn mat_color_format(mat: &Mat) -> ColorFormat {
    match mat.channels() {
        1 => ColorFormat::Gray,
        3 => ColorFormat::Bgr,
        4 => ColorFormat::Rgba,
        ch => {
            // 지원하지 않는 채널 수는 경고를 남기고 BGR로 폴백한다.
            eprintln!("[GUI] Unsupported channel count for preview: {}", ch);
            ColorFormat::Bgr
        }
    }
}
