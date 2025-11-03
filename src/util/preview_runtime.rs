//! SDL 기반 프리뷰 GUI를 백그라운드 스레드에서 실행한다.
//! 카메라/전처리/Bird-eye 프레임을 수신해 윈도우에 표시하고, 종료 이벤트를 전달한다.

use crate::rte::rte_dto::{CameraBuffer, ColorFormat};
use crate::util::preview_window::SdlPreview;
use crate::util::sdl_env::SdlEnv;
use opencv::core::{Mat, Size};
use opencv::imgproc;
use opencv::prelude::MatTraitConstManual;
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Keycode;
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

/// 프리뷰 스레드 핸들과 통신 채널을 보관한다.
pub struct PreviewRuntime {
    pub tx: mpsc::Sender<PreviewMessage>,
    pub event_rx: mpsc::Receiver<PreviewEvent>,
    pub handle: thread::JoinHandle<()>,
}

/// 프레임 데이터를 전송할 때 사용하는 패킷 구조체.
pub struct FramePacket {
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub format: ColorFormat,
    pub payload: FramePayload,
}

/// 프레임 데이터가 저장된 방식.
pub enum FramePayload {
    Camera(Arc<CameraBuffer>),
    Mat(Arc<Mat>),
    Owned(Vec<u8>),
}

/// 프리뷰 스레드 입력 메시지 종류.
pub enum PreviewMessage {
    Raw(FramePacket),
    Processed(FramePacket),
    Bird(FramePacket),
    Path(FramePacket),
}

/// 프리뷰 스레드가 메인 루프로 전달하는 이벤트.
pub enum PreviewEvent {
    Quit,
}

/// SDL 프리뷰 스레드를 생성하고 통신 채널을 돌려준다.
pub fn spawn_preview_thread() -> opencv::Result<PreviewRuntime> {
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

    Ok(PreviewRuntime {
        tx,
        event_rx,
        handle,
    })
}

/// SDL 프리뷰 스레드 본체.
/// - 수신 큐에서 최신 프레임을 가져와 윈도우에 표시한다.
/// - 사용자의 키/윈도우 이벤트를 처리해 종료 여부를 판단한다.
fn run_preview_thread(
    rx: mpsc::Receiver<PreviewMessage>,
    event_tx: mpsc::Sender<PreviewEvent>,
) -> opencv::Result<()> {
    let mut env = SdlEnv::new()?;
    let mut raw_preview: Option<SdlPreview> = None;
    let mut processed_preview: Option<SdlPreview> = None;
    let mut birds_eye_preview: Option<SdlPreview> = None;
    let mut path_preview: Option<SdlPreview> = None;
    let mut raw_enabled = true;
    let mut processed_enabled = true;
    let mut birds_enabled = true;
    let mut path_enabled = true;
    let mut running = true;

    let mut pending_raw: Option<FramePacket> = None;
    let mut pending_processed: Option<FramePacket> = None;
    let mut pending_bird: Option<FramePacket> = None;
    let mut pending_path: Option<FramePacket> = None;
    let mut last_present = Instant::now();
    const WINDOW_MARGIN: i32 = 40;
    const WINDOW_WIDTH: i32 = 640;
    const WINDOW_HEIGHT: i32 = 480;
    const RAW_WINDOW_POS: (i32, i32) = (WINDOW_MARGIN, WINDOW_MARGIN);
    const PROCESSED_WINDOW_POS: (i32, i32) =
        (WINDOW_MARGIN + WINDOW_WIDTH + WINDOW_MARGIN, WINDOW_MARGIN);
    const BIRD_WINDOW_POS: (i32, i32) =
        (WINDOW_MARGIN, WINDOW_MARGIN + WINDOW_HEIGHT + WINDOW_MARGIN);
    const PATH_WINDOW_POS: (i32, i32) = (
        WINDOW_MARGIN + WINDOW_WIDTH * 2 + WINDOW_MARGIN * 3,
        WINDOW_MARGIN,
    );

    if raw_enabled {
        let dummy = FramePacket {
            width: 640,
            height: 480,
            stride: 640 * 3,
            format: ColorFormat::Bgr,
            payload: FramePayload::Owned(vec![0; (640 * 480 * 3) as usize]),
        };
        if ensure_preview(
            &mut raw_preview,
            &env,
            "Raw View",
            &dummy,
            Some(RAW_WINDOW_POS),
        )
        .is_err()
        {
            raw_enabled = false;
        }
    }
    if let Some(preview) = raw_preview.as_mut() {
        preview.raise();
        thread::sleep(Duration::from_millis(50));
    }
    if processed_enabled {
        let dummy = FramePacket {
            width: 640,
            height: 480,
            stride: 640,
            format: ColorFormat::Gray,
            payload: FramePayload::Owned(vec![0; (640 * 480) as usize]),
        };
        if ensure_preview(
            &mut processed_preview,
            &env,
            "Processed View",
            &dummy,
            Some(PROCESSED_WINDOW_POS),
        )
        .is_err()
        {
            processed_enabled = false;
        }
    }
    if let Some(preview) = processed_preview.as_mut() {
        preview.raise();
        thread::sleep(Duration::from_millis(50));
    }
    if birds_enabled {
        let dummy = FramePacket {
            width: 640,
            height: 480,
            stride: 640,
            format: ColorFormat::Gray,
            payload: FramePayload::Owned(vec![0; (640 * 480) as usize]),
        };
        if ensure_preview(
            &mut birds_eye_preview,
            &env,
            "Bird's Eye View",
            &dummy,
            Some(BIRD_WINDOW_POS),
        )
        .is_err()
        {
            birds_enabled = false;
        }
    }
    if let Some(preview) = birds_eye_preview.as_mut() {
        preview.raise();
        thread::sleep(Duration::from_millis(50));
    }
    if let Some(preview) = raw_preview.as_mut() {
        preview.raise();
    }

    while running {
        match rx.recv_timeout(Duration::from_millis(16)) {
            Ok(msg) => {
                update_pending_messages(
                    msg,
                    &mut pending_raw,
                    &mut pending_processed,
                    &mut pending_bird,
                    &mut pending_path,
                );
                while let Ok(msg) = rx.try_recv() {
                    update_pending_messages(
                        msg,
                        &mut pending_raw,
                        &mut pending_processed,
                        &mut pending_bird,
                        &mut pending_path,
                    );
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        let now = Instant::now();
        let frame_interval = Duration::from_millis(66);
        let should_present = now.duration_since(last_present) >= frame_interval;

        if should_present {
            if raw_enabled {
                if let Some(packet) = pending_raw.take() {
                    if ensure_preview(
                        &mut raw_preview,
                        &env,
                        "Raw View",
                        &packet,
                        Some(RAW_WINDOW_POS),
                    )
                    .is_ok()
                    {
                        present_packet(raw_preview.as_mut(), &packet, true);
                    }
                }
            } else {
                raw_preview = None;
            }

            if processed_enabled {
                if let Some(packet) = pending_processed.take() {
                    if ensure_preview(
                        &mut processed_preview,
                        &env,
                        "Processed View",
                        &packet,
                        Some(PROCESSED_WINDOW_POS),
                    )
                    .is_ok()
                    {
                        present_packet(processed_preview.as_mut(), &packet, false);
                    }
                }
            } else {
                processed_preview = None;
            }

            if birds_enabled {
                if let Some(packet) = pending_bird.take() {
                    if ensure_preview(
                        &mut birds_eye_preview,
                        &env,
                        "Bird's Eye View",
                        &packet,
                        Some(BIRD_WINDOW_POS),
                    )
                    .is_ok()
                    {
                        present_packet(birds_eye_preview.as_mut(), &packet, false);
                    }
                }
            } else {
                birds_eye_preview = None;
            }

            if path_enabled {
                if let Some(packet) = pending_path.take() {
                    if ensure_preview(
                        &mut path_preview,
                        &env,
                        "Path View",
                        &packet,
                        Some(PATH_WINDOW_POS),
                    )
                    .is_ok()
                    {
                        present_packet(path_preview.as_mut(), &packet, false);
                    }
                }
            } else {
                path_preview = None;
            }

            if raw_enabled || processed_enabled || birds_enabled || path_enabled {
                last_present = now;
            }
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
                Event::KeyDown {
                    keycode: Some(Keycode::M),
                    ..
                } => {
                    path_enabled = !path_enabled;
                    path_preview = None;
                    println!(
                        "[GUI] Path preview {}",
                        if path_enabled { "enabled" } else { "disabled" }
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
                    } else if path_preview.as_ref().map(|p| p.window_id()) == Some(window_id) {
                        path_preview = None;
                        path_enabled = false;
                        println!("[GUI] Path preview window closed");
                    }
                }
                _ => {}
            }
        }
    }

    Ok(())
}

/// 수신된 메시지를 최신 패킷으로 교체해 나중에 렌더링하도록 저장한다.
fn update_pending_messages(
    msg: PreviewMessage,
    pending_raw: &mut Option<FramePacket>,
    pending_processed: &mut Option<FramePacket>,
    pending_bird: &mut Option<FramePacket>,
    pending_path: &mut Option<FramePacket>,
) {
    match msg {
        PreviewMessage::Raw(frame) => *pending_raw = Some(frame),
        PreviewMessage::Processed(frame) => *pending_processed = Some(frame),
        PreviewMessage::Bird(frame) => *pending_bird = Some(frame),
        PreviewMessage::Path(frame) => *pending_path = Some(frame),
    }
}

/// 윈도우가 생성되지 않았다면 새로 만들고, 존재하면 재사용한다.
fn ensure_preview(
    target: &mut Option<SdlPreview>,
    env: &SdlEnv,
    title: &str,
    frame: &FramePacket,
    position: Option<(i32, i32)>,
) -> opencv::Result<()> {
    if target.is_none() {
        let preview = SdlPreview::new(
            &env.video,
            title,
            frame.width,
            frame.height,
            frame.format,
            position,
        )?;
        *target = Some(preview);
    }
    Ok(())
}

/// 준비된 패킷을 SDL 프리뷰에 렌더링한다.
fn present_packet(preview: Option<&mut SdlPreview>, packet: &FramePacket, is_raw: bool) {
    if let Some(preview) = preview {
        match &packet.payload {
            FramePayload::Camera(buffer) => {
                if let Err(err) = preview.present(
                    packet.width,
                    packet.height,
                    packet.format,
                    buffer.as_slice(),
                    packet.stride,
                ) {
                    eprintln!("[GUI] Failed to present frame: {}", err);
                }
            }
            FramePayload::Mat(mat) => {
                if let Err(err) = render_resized_mat(preview, mat.as_ref(), packet.format, is_raw) {
                    eprintln!("[GUI] Failed to present frame: {}", err);
                }
            }
            FramePayload::Owned(data) => {
                let stride = if packet.format == ColorFormat::Gray {
                    packet.width as usize
                } else {
                    packet.width as usize * 3
                };
                if let Err(err) =
                    preview.present(packet.width, packet.height, packet.format, data, stride)
                {
                    eprintln!("[GUI] Failed to present frame: {}", err);
                }
            }
        }
    }
}

/// OpenCV `Mat`을 프리뷰 크기에 맞게 리사이즈해 출력한다.
fn render_resized_mat(
    preview: &mut SdlPreview,
    mat: &Mat,
    format: ColorFormat,
    is_raw: bool,
) -> opencv::Result<()> {
    let target_size = if is_raw {
        Size::new(320, 240)
    } else {
        Size::new(320, 240)
    };
    let mut resized = Mat::default();
    imgproc::resize(
        mat,
        &mut resized,
        target_size,
        0.0,
        0.0,
        imgproc::INTER_LINEAR,
    )?;
    let data = resized.data_bytes()?;
    let stride = if format == ColorFormat::Gray {
        target_size.width as usize
    } else {
        target_size.width as usize * 3
    };
    preview.present(
        target_size.width as u32,
        target_size.height as u32,
        format,
        data,
        stride,
    )
}
