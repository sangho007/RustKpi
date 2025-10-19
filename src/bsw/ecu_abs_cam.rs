use crate::bsw::lib::cam_lib;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::{Result, videoio};
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::task;

const CAPTURE_QUEUE_DEPTH: usize = 3;

pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    let CameraChannels { raw_tx, .. } = camera;
    let (frame_tx, mut frame_rx) = mpsc::channel::<cam_lib::CapturedFrame>(CAPTURE_QUEUE_DEPTH);

    let capture_thread = std::thread::Builder::new()
        .name("camera-capture".to_string())
        .spawn(move || camera_capture_loop(frame_tx))?;

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

fn camera_capture_loop(mut frame_tx: mpsc::Sender<cam_lib::CapturedFrame>) -> Result<()> {
    // 비디오 파일이나 카메라 장치를 엽니다.
    let cammode = true;
    let mut cap: Box<dyn cam_lib::FrameCapture> = if cammode {
        let libcam =
            cam_lib::libcamera_capture::LibcameraCapture::new(1280, 720, 30).map_err(|e| {
                opencv::Error::new(
                    opencv::core::StsError,
                    format!("Failed to initialize libcamera: {}", e),
                )
            })?;
        Box::new(libcam)
    } else {
        let file_cap = videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?;
        Box::new(file_cap)
    };

    loop {
        match cap.read_frame() {
            Ok(Some(captured)) => {
                if frame_tx.blocking_send(captured).is_err() {
                    break;
                }
            }
            Ok(None) => {
                println!("[bsw] 비디오 스트림 종료.");
                break;
            }
            Err(e) => {
                eprintln!("[bsw] 에러: 프레임 읽기 실패: {:?}", e);
                return Err(e);
            }
        }
    }

    Ok(())
}
