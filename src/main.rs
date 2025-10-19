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
use std::sync::Arc;
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

#[tokio::main(flavor = "multi_thread")]
async fn main() -> opencv::Result<()> {
    let RteSystem { channels } = rte::rte_main::init();
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();
    let _control_channels = channels.control.clone();

    // Enable OpenCL acceleration paths if the platform supports it (no-op otherwise).
    if let Err(err) = core::set_use_opencl(true) {
        eprintln!("[INIT] Failed to enable OpenCL: {err}");
    }

    // BSW Task 생성
    tokio::spawn(bsw::ecu_abs_cam::ea_cam_provider(camera_channels.clone()));
    // tokio::spawn(bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(
    //     ultrasonic_channels.clone(),
    // ));
    // tokio::spawn(bsw::ecu_abs_pwm::ea_pca9685_actuator(
    //     "MotorControl",
    //     control_channels.clone(),
    // ));

    // ASW Task 생성
    tokio::spawn(asw::vs_lane::runnable_pre_processing(
        "PreProcess",
        camera_channels.clone(),
    ));
    tokio::spawn(asw::vs_lane::runnable_get_lane_angle(
        "LaneAngle",
        camera_channels.clone(),
    ));
    // tokio::spawn(asw::forwardcollision_ultrasonic::runnable_obstacle_detection(
    //     "UssObstacle",
    //     ultrasonic_channels.clone(),
    // ));
    // tokio::spawn(asw::vs_trafficlight::runnable_trafficlight_detection(
    //     "TrafficLightDetection",
    //     camera_channels.clone(),
    // ));

    // 디버깅용 코드
    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

    const DEBUG_ON: bool = true;

    let mut camraw_rx = camera_channels.raw_tx.subscribe();
    let mut processed_rx = camera_channels.processed_tx.subscribe();
    let mut birds_eye_rx = camera_channels.bird_eye_tx.subscribe();
    let mut lane_angle_rx = camera_channels.lane_angle_tx.subscribe();
    let mut distance_rx = ultrasonic_channels.raw_tx.subscribe();

    // 각 창에 표시할 최신 프레임을 저장할 변수 (루프 외부에 선언)
    let mut latest_raw_frame: Option<Arc<DtoCamRaw>> = None;
    let mut latest_processed_frame: Option<Arc<DtoCamProcessed>> = None;
    let mut latest_birds_eye_frame: Option<Arc<DtoCamBirdEyeView>> = None;
    let mut raw_preview: Option<SdlPreview> = None;
    let mut processed_preview: Option<SdlPreview> = None;
    let mut birds_eye_preview: Option<SdlPreview> = None;
    let mut raw_preview_enabled = true;
    let mut processed_preview_enabled = true;
    let mut birds_eye_preview_enabled = true;
    let mut sdl_env = if DEBUG_ON { Some(SdlEnv::new()?) } else { None };

    // Main 스레드에서 GUI 이벤트 루프 실행
    'main_loop: loop {
        select! {
            biased;
            result = camraw_rx.recv() => match result {
                Ok(camraw) => {
                    let mut newest = camraw;
                    while let Ok(newer) = camraw_rx.try_recv() {
                        newest = newer;
                    }
                    latest_raw_frame = Some(newest);
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
                    latest_processed_frame = Some(newest);
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
                    latest_birds_eye_frame = Some(newest);
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
            }
        }

        if DEBUG_ON {
            while let Ok(newer) = camraw_rx.try_recv() {
                latest_raw_frame = Some(newer);
            }

            let env = match sdl_env.as_mut() {
                Some(env) => env,
                None => continue,
            };

            if !raw_preview_enabled {
                raw_preview = None;
            } else if let Some(frame) = &latest_raw_frame {
                if raw_preview.is_none() {
                    raw_preview = Some(SdlPreview::new(
                        &env.video,
                        "Raw View",
                        frame.width,
                        frame.height,
                        frame.color_format,
                    )?);
                }

                if let Some(preview) = raw_preview.as_mut() {
                    preview.present(
                        frame.width,
                        frame.height,
                        frame.color_format,
                        frame.buffer.as_slice(),
                        frame.stride,
                    )?;
                }
            }

            if !processed_preview_enabled {
                processed_preview = None;
            } else if let Some(processed) = &latest_processed_frame {
                let mat = processed.img.as_ref();
                let data = mat.data_bytes()?;
                let stride = mat.step1(0)? as usize;
                let format = mat_color_format(mat);

                if processed_preview.is_none() {
                    processed_preview = Some(SdlPreview::new(
                        &env.video,
                        "Processed View",
                        processed.width,
                        processed.height,
                        format,
                    )?);
                }

                if let Some(preview) = processed_preview.as_mut() {
                    preview.present(processed.width, processed.height, format, data, stride)?;
                }
            }

            if !birds_eye_preview_enabled {
                birds_eye_preview = None;
            } else if let Some(birds_eye) = &latest_birds_eye_frame {
                let mat = birds_eye.img.as_ref();
                let data = mat.data_bytes()?;
                let stride = mat.step1(0)? as usize;
                let format = mat_color_format(mat);

                if birds_eye_preview.is_none() {
                    birds_eye_preview = Some(SdlPreview::new(
                        &env.video,
                        "Bird's Eye View",
                        birds_eye.width,
                        birds_eye.height,
                        format,
                    )?);
                }

                if let Some(preview) = birds_eye_preview.as_mut() {
                    preview.present(birds_eye.width, birds_eye.height, format, data, stride)?;
                }
            }

            let mut should_quit = false;
            for event in env.event_pump.poll_iter() {
                match event {
                    Event::Quit { .. }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => {
                        should_quit = true;
                    }
                    Event::KeyDown {
                        keycode: Some(Keycode::R),
                        ..
                    } => {
                        raw_preview_enabled = !raw_preview_enabled;
                        raw_preview = None;
                        println!(
                            "[GUI] Raw preview {}",
                            if raw_preview_enabled {
                                "enabled"
                            } else {
                                "disabled"
                            }
                        );
                    }
                    Event::KeyDown {
                        keycode: Some(Keycode::P),
                        ..
                    } => {
                        processed_preview_enabled = !processed_preview_enabled;
                        processed_preview = None;
                        println!(
                            "[GUI] Processed preview {}",
                            if processed_preview_enabled {
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
                        birds_eye_preview_enabled = !birds_eye_preview_enabled;
                        birds_eye_preview = None;
                        println!(
                            "[GUI] Bird's eye preview {}",
                            if birds_eye_preview_enabled {
                                "enabled"
                            } else {
                                "disabled"
                            }
                        );
                    }
                    Event::Window {
                        win_event: WindowEvent::Close,
                        window_id,
                        ..
                    } => {
                        if raw_preview.as_ref().map(|p| p.window_id()) == Some(window_id) {
                            raw_preview = None;
                            raw_preview_enabled = false;
                            println!("[GUI] Raw preview window closed");
                        } else if processed_preview.as_ref().map(|p| p.window_id())
                            == Some(window_id)
                        {
                            processed_preview = None;
                            processed_preview_enabled = false;
                            println!("[GUI] Processed preview window closed");
                        } else if birds_eye_preview.as_ref().map(|p| p.window_id())
                            == Some(window_id)
                        {
                            birds_eye_preview = None;
                            birds_eye_preview_enabled = false;
                            println!("[GUI] Bird's eye preview window closed");
                        }
                    }
                    _ => {}
                }
            }

            if should_quit {
                break;
            }
        }
    }

    println!("== 시스템 실행 중... (Ctrl+C로 종료) ==");
    tokio::signal::ctrl_c()
        .await
        .expect("Ctrl-C 핸들러 설정 실패");
    println!("\n== 시뮬레이션 종료 ==");
    Ok(())
}
