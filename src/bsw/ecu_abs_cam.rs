//! BSW ECU in charge of acquiring camera frames and pushing them onto the RTE.

use crate::bsw::lib::cam_lib;
use crate::calibration::camera::CameraCalibration;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::prelude::VideoCaptureTrait;
use opencv::{Result, videoio};
use std::path::Path;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tokio::sync::mpsc;
use tokio::task;

/// Spawn the camera provider tasks and shuttle frames into the raw RTE channel.
pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    let camera_calibration = CameraCalibration::default();
    let CameraChannels { raw_tx, .. } = camera;
    let (frame_tx, mut frame_rx) =
        mpsc::channel::<cam_lib::CapturedFrame>(camera_calibration.capture_queue_depth);

    let capture_config = camera_calibration;
    let capture_thread = std::thread::Builder::new()
        .name("camera-capture".to_string())
        .spawn(move || camera_capture_loop(frame_tx, capture_config))
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Failed to spawn camera capture thread: {}", e),
            )
        })?;

    let mut alive_cnt = 0u32;

    while let Some(captured) = frame_rx.recv().await {
        // DTO로 다시 패키징하여 RTE 카메라 RAW 채널로 전달한다.
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

/// Blocking capture loop executed on a dedicated thread. It keeps trying to
/// initialise the selected capture backend and pushes frames onto the channel
/// until the receiver goes away.
fn camera_capture_loop(
    frame_tx: mpsc::Sender<cam_lib::CapturedFrame>,
    camera_calibration: CameraCalibration,
) -> Result<()> {
    let mut next_frame_due = Instant::now();
    let frame_interval = camera_calibration.frame_interval();
    // 계속해서 캡처 백엔드를 재초기화해 스트림 끊김에 대응한다.
    loop {
        let mut cap = match init_capture(camera_calibration) {
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
                    let now = Instant::now();
                    if now < next_frame_due {
                        thread::sleep(next_frame_due - now);
                    } else {
                        next_frame_due = now;
                    }
                    if frame_tx.blocking_send(captured).is_err() {
                        return Ok(());
                    }
                    next_frame_due += frame_interval;
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

/// Select the active capture backend. When `cammode` is true we use the
/// libcamera bridge; otherwise we fall back to the sample video file.
fn init_capture(camera_calibration: CameraCalibration) -> Result<Box<dyn cam_lib::FrameCapture>> {
    let cammode = camera_calibration.use_libcamera;
    if cammode {
        let libcam = cam_lib::libcamera_capture::LibcameraCapture::new(
            camera_calibration.width_u32(),
            camera_calibration.height_u32(),
            camera_calibration.target_fps,
        )?;
        Ok(Box::new(libcam))
    } else {
        let preferred_path = Path::new(camera_calibration.sample_video_preferred);
        let sample_path = if preferred_path.exists() {
            camera_calibration.sample_video_preferred
        } else {
            camera_calibration.sample_video_fallback
        };

        let mut file_cap = videoio::VideoCapture::from_file(sample_path, videoio::CAP_ANY)?;
        let _ = file_cap.set(
            videoio::CAP_PROP_FRAME_WIDTH,
            camera_calibration.width as f64,
        );
        let _ = file_cap.set(
            videoio::CAP_PROP_FRAME_HEIGHT,
            camera_calibration.height as f64,
        );
        Ok(Box::new(file_cap))
    }
}
