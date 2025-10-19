// main.rs

mod asw;
mod bsw;
mod calibration;
mod gui;
mod rte;

use crate::rte::rte_dto::*;
use crate::rte::rte_main::RteSystem;
use gui::sdl_env::SdlEnv;
use gui::sdl_preview::SdlPreview;
use opencv::core;
use opencv::core::Mat;
use opencv::prelude::{MatTraitConst, MatTraitConstManual};
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Keycode;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use tokio::{select, sync::broadcast::error::RecvError};

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

struct FramePacket {
    width: u32,
    height: u32,
    stride: usize,
    format: ColorFormat,
    data: Vec<u8>,
}

enum PreviewMessage {
    Raw(FramePacket),
    Processed(FramePacket),
    Bird(FramePacket),
}

enum PreviewEvent {
    Quit,
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> opencv::Result<()> {
    let RteSystem { channels } = rte::rte_main::init();
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();
    let _control_channels = channels.control.clone();

    if let Err(err) = core::set_use_opencl(true) {
        eprintln!("[INIT] Failed to enable OpenCL: {err}");
    }

    tokio::spawn(bsw::ecu_abs_cam::ea_cam_provider(camera_channels.clone()));
    tokio::spawn(asw::vs_lane::runnable_pre_processing(
        "PreProcess",
        camera_channels.clone(),
    ));
    tokio::spawn(asw::vs_lane::runnable_get_lane_angle(
        "LaneAngle",
        camera_channels.clone(),
    ));

    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

    const DEBUG_ON: bool = true;

    let (mut preview_tx, mut preview_event_rx, preview_handle) = if DEBUG_ON {
        let (tx, rx) = mpsc::channel::<PreviewMessage>();
        let (event_tx, event_rx) = mpsc::channel::<PreviewEvent>();
        let handle = thread::Builder::new()
            .name("sdl-preview".to_string())
            .spawn(move || {
                if let Err(err) = run_preview_thread(rx, event_tx) {
                    eprintln!("[GUI] preview thread error: {}", err);
                }
            })
            .map_err(|e| {
                opencv::Error::new(
                    opencv::core::StsError,
                    format!("Failed to spawn preview thread: {}", e),
                )
            })?;
        (Some(tx), Some(event_rx), Some(handle))
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
                            data: newest.buffer.as_slice().to_vec(),
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
                        let mat = newest.img.as_ref();
                        match (mat.data_bytes(), mat.step1(0)) {
                            (Ok(data), Ok(stride)) => {
                                let format = mat_color_format(mat);
                                let payload = FramePacket {
                                    width: newest.width,
                                    height: newest.height,
                                    stride: stride as usize,
                                    format,
                                    data: data.to_vec(),
                                };
                                let _ = tx.send(PreviewMessage::Processed(payload));
                            }
                            (Err(err), _) => eprintln!("[GUI] Failed to read processed data: {}", err),
                            (_, Err(err)) => eprintln!("[GUI] Failed to read processed stride: {}", err),
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
                        let mat = newest.img.as_ref();
                        match (mat.data_bytes(), mat.step1(0)) {
                            (Ok(data), Ok(stride)) => {
                                let format = mat_color_format(mat);
                                let payload = FramePacket {
                                    width: newest.width,
                                    height: newest.height,
                                    stride: stride as usize,
                                    format,
                                    data: data.to_vec(),
                                };
                                let _ = tx.send(PreviewMessage::Bird(payload));
                            }
                            (Err(err), _) => eprintln!("[GUI] Failed to read bird-eye data: {}", err),
                            (_, Err(err)) => eprintln!("[GUI] Failed to read bird-eye stride: {}", err),
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

fn run_preview_thread(
    rx: mpsc::Receiver<PreviewMessage>,
    event_tx: mpsc::Sender<PreviewEvent>,
) -> opencv::Result<()> {
    let mut env = SdlEnv::new()?;
    let mut raw_preview: Option<SdlPreview> = None;
    let mut processed_preview: Option<SdlPreview> = None;
    let mut birds_eye_preview: Option<SdlPreview> = None;
    let mut raw_enabled = true;
    let mut processed_enabled = true;
    let mut birds_enabled = true;
    let mut running = true;

    let mut pending_raw: Option<FramePacket> = None;
    let mut pending_processed: Option<FramePacket> = None;
    let mut pending_bird: Option<FramePacket> = None;

    while running {
        match rx.recv_timeout(Duration::from_millis(16)) {
            Ok(msg) => {
                update_pending_messages(
                    msg,
                    &mut pending_raw,
                    &mut pending_processed,
                    &mut pending_bird,
                );
                while let Ok(msg) = rx.try_recv() {
                    update_pending_messages(
                        msg,
                        &mut pending_raw,
                        &mut pending_processed,
                        &mut pending_bird,
                    );
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        if raw_enabled {
            if let Some(packet) = pending_raw.take() {
                if ensure_preview(&mut raw_preview, &env, "Raw View", &packet).is_ok() {
                    present_packet(raw_preview.as_mut(), &packet);
                }
            }
        } else {
            raw_preview = None;
        }

        if processed_enabled {
            if let Some(packet) = pending_processed.take() {
                if ensure_preview(&mut processed_preview, &env, "Processed View", &packet).is_ok() {
                    present_packet(processed_preview.as_mut(), &packet);
                }
            }
        } else {
            processed_preview = None;
        }

        if birds_enabled {
            if let Some(packet) = pending_bird.take() {
                if ensure_preview(&mut birds_eye_preview, &env, "Bird's Eye View", &packet).is_ok()
                {
                    present_packet(birds_eye_preview.as_mut(), &packet);
                }
            }
        } else {
            birds_eye_preview = None;
        }

        for event in env.event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => {
                    let _ = event_tx.send(PreviewEvent::Quit);
                    running = false;
                    break;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::R),
                    ..
                } => {
                    raw_enabled = !raw_enabled;
                    raw_preview = None;
                    println!(
                        "[GUI] Raw preview {}",
                        if raw_enabled { "enabled" } else { "disabled" }
                    );
                }
                Event::KeyDown {
                    keycode: Some(Keycode::P),
                    ..
                } => {
                    processed_enabled = !processed_enabled;
                    processed_preview = None;
                    println!(
                        "[GUI] Processed preview {}",
                        if processed_enabled {
                            "enabled"
                        } else {
                            "disabled"
                        }
                    );
                }
                Event::KeyDown {
                    keycode: Some(Keycode::B),
                    ..
                } => {
                    birds_enabled = !birds_enabled;
                    birds_eye_preview = None;
                    println!(
                        "[GUI] Bird's eye preview {}",
                        if birds_enabled { "enabled" } else { "disabled" }
                    );
                }
                Event::Window {
                    win_event: WindowEvent::Close,
                    window_id,
                    ..
                } => {
                    if raw_preview.as_ref().map(|p| p.window_id()) == Some(window_id) {
                        raw_preview = None;
                        raw_enabled = false;
                        println!("[GUI] Raw preview window closed");
                    } else if processed_preview.as_ref().map(|p| p.window_id()) == Some(window_id) {
                        processed_preview = None;
                        processed_enabled = false;
                        println!("[GUI] Processed preview window closed");
                    } else if birds_eye_preview.as_ref().map(|p| p.window_id()) == Some(window_id) {
                        birds_eye_preview = None;
                        birds_enabled = false;
                        println!("[GUI] Bird's eye preview window closed");
                    }
                }
                _ => {}
            }
        }
    }

    Ok(())
}

fn update_pending_messages(
    msg: PreviewMessage,
    pending_raw: &mut Option<FramePacket>,
    pending_processed: &mut Option<FramePacket>,
    pending_bird: &mut Option<FramePacket>,
) {
    match msg {
        PreviewMessage::Raw(frame) => *pending_raw = Some(frame),
        PreviewMessage::Processed(frame) => *pending_processed = Some(frame),
        PreviewMessage::Bird(frame) => *pending_bird = Some(frame),
    }
}

fn ensure_preview(
    target: &mut Option<SdlPreview>,
    env: &SdlEnv,
    title: &str,
    frame: &FramePacket,
) -> opencv::Result<()> {
    if target.is_none() {
        let preview = SdlPreview::new(&env.video, title, frame.width, frame.height, frame.format)?;
        *target = Some(preview);
    }
    Ok(())
}

fn present_packet(preview: Option<&mut SdlPreview>, packet: &FramePacket) {
    if let Some(preview) = preview {
        if let Err(err) = preview.present(
            packet.width,
            packet.height,
            packet.format,
            &packet.data,
            packet.stride,
        ) {
            eprintln!("[GUI] Failed to present frame: {}", err);
        }
    }
}
