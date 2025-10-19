use crate::util::preview_runtime::{
    self,
    FramePacket,
    FramePayload,
    PreviewEvent,
    PreviewMessage,
};
use crate::rte::rte_dto::*;
use crate::rte::rte_main::RteChannels;
use opencv::core::Mat;
use opencv::prelude::MatTraitConst;
use tokio::{select, sync::broadcast::error::RecvError};

const DEBUG_ON: bool = true;

pub async fn run(channels: RteChannels) -> opencv::Result<()> {
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();

    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

    let (mut preview_tx, mut preview_event_rx, preview_handle) = if DEBUG_ON {
        let runtime = preview_runtime::spawn_preview_thread()?;
        (Some(runtime.tx), Some(runtime.event_rx), Some(runtime.handle))
    } else {
        (None, None, None)
    };

    let mut camraw_rx = camera_channels.raw_tx.subscribe();
    let mut processed_rx = camera_channels.processed_tx.subscribe();
    let mut birds_eye_rx = camera_channels.bird_eye_tx.subscribe();
    let mut lane_angle_rx = camera_channels.lane_angle_tx.subscribe();
    let mut distance_rx = ultrasonic_channels.raw_tx.subscribe();

    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    'main_loop: loop {
        select! {
            biased;
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
                            Err(err) => eprintln!("[GUI] Failed to read processed stride: {}", err),
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
                            Err(err) => eprintln!("[GUI] Failed to read bird-eye stride: {}", err),
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
            result = lane_angle_rx.recv() => match result {
                Ok(lane_angle) => {
                    println!("Angle: {}, alive_cnt: {}", lane_angle.angle, lane_angle.alive_cnt);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Lane angle lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Lane angle channel closed.");
                    break 'main_loop;
                }
            },
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
            result = &mut ctrl_c => {
                if let Err(err) = result {
                    eprintln!("[MAIN] Failed to receive Ctrl-C signal: {}", err);
                } else {
                    println!("[MAIN] Ctrl-C received, shutting down...");
                }
                break 'main_loop;
            },
        }

        if let Some(event_rx) = preview_event_rx.as_mut() {
            if let Ok(PreviewEvent::Quit) = event_rx.try_recv() {
                break 'main_loop;
            }
        }
    }

    if let Some(tx) = preview_tx.take() {
        drop(tx);
    }
    if let Some(handle) = preview_handle {
        let _ = handle.join();
    }

    println!("== 시뮬레이션 종료 ==");
    Ok(())
}

fn mat_color_format(mat: &Mat) -> ColorFormat {
    match mat.channels() {
        1 => ColorFormat::Gray,
        3 => ColorFormat::Bgr,
        4 => ColorFormat::Rgba,
        ch => {
            eprintln!("[GUI] Unsupported channel count for preview: {}", ch);
            ColorFormat::Bgr
        }
    }
}
