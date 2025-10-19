use crate::bsw::lib::cam_lib;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::{Result, videoio};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::task;

const CAPTURE_QUEUE_DEPTH: usize = 3;

pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    let CameraChannels { raw_tx, .. } = camera;
    let (frame_tx, mut frame_rx) = mpsc::channel::<cam_lib::CapturedFrame>(CAPTURE_QUEUE_DEPTH);

    let capture_thread = std::thread::Builder::new()
        .name("camera-capture".to_string())
        .spawn(move || camera_capture_loop(frame_tx))
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Failed to spawn camera capture thread: {}", e),
            )
        })?;

    let mut alive_cnt = 0u32;

    while let Some(captured) = frame_rx.recv().await {
        let cam_raw = Arc::new(DtoCamRaw::new(
            captured.buffer,
            captured.width,
            captured.height,
            captured.stride,
            captured.bytes_per_pixel,
            alive_cnt,
            captured.color_format,
        ));

        let _ = raw_tx.send(cam_raw);
        alive_cnt = alive_cnt.wrapping_add(1);
    }

    let join_result = task::spawn_blocking(move || capture_thread.join())
        .await
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Camera capture join task failed: {}", e),
            )
        })?;

    match join_result {
        Ok(Ok(())) => Ok(()),
        Ok(Err(err)) => Err(err),
        Err(_) => Err(opencv::Error::new(
            opencv::core::StsError,
            "Camera capture thread panicked",
        )),
    }
}

fn camera_capture_loop(frame_tx: mpsc::Sender<cam_lib::CapturedFrame>) -> Result<()> {
    loop {
        let mut cap = match init_capture() {
            Ok(cap) => cap,
            Err(e) => {
                eprintln!("[bsw] camera init failed: {e:?}. retrying...");
                if frame_tx.is_closed() {
                    return Ok(());
                }
                thread::sleep(Duration::from_millis(500));
                continue;
            }
        };

        loop {
            match cap.read_frame() {
                Ok(Some(captured)) => {
                    if frame_tx.blocking_send(captured).is_err() {
                        return Ok(());
                    }
                }
                Ok(None) => {
                    println!("[bsw] 비디오 스트림 종료. 재시도합니다.");
                    break;
                }
                Err(e) => {
                    eprintln!("[bsw] 프레임 읽기 실패: {e:?}. 캡처를 재시도합니다.");
                    break;
                }
            }
        }

        if frame_tx.is_closed() {
            return Ok(());
        }

        thread::sleep(Duration::from_millis(500));
    }
}

fn init_capture() -> Result<Box<dyn cam_lib::FrameCapture>> {
    let cammode = true;
    if cammode {
        let libcam = cam_lib::libcamera_capture::LibcameraCapture::new(1280, 720, 30)?;
        Ok(Box::new(libcam))
    } else {
        let file_cap = videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?;
        Ok(Box::new(file_cap))
    }
}
