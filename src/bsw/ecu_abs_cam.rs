use crate::bsw::lib::cam_lib;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::core::Mat;
use opencv::prelude::{MatTraitConst, VideoCaptureTraitConst};
use opencv::{videoio, Result};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    tokio::task::spawn_blocking(move || -> Result<()> {
        let CameraChannels { raw_tx, .. } = camera;
        let mut alive_cnt = 0;
        
        // 비디오 파일이나 카메라 장치를 엽니다.
        let cammode = true;
        let (mut cap, frame_width, frame_height): (Box<dyn cam_lib::FrameCapture>, u32, u32) = if cammode {
            let libcam = cam_lib::libcamera_capture::LibcameraCapture::new(1280, 720, 30)
                .map_err(|e| opencv::Error::new(
                    opencv::core::StsError,
                    format!("Failed to initialize libcamera: {}", e),
                ))?;
            let (width, height) = libcam.dimensions();
            (Box::new(libcam), width, height)
        } else {
            // videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?
            let file_cap = videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?;
            let width = file_cap.get(videoio::CAP_PROP_FRAME_WIDTH)? as u32;
            let height = file_cap.get(videoio::CAP_PROP_FRAME_HEIGHT)? as u32;
            let width = if width == 0 { 1280 } else { width };
            let height = if height == 0 { 720 } else { height };
            (Box::new(file_cap), width, height)
        };
        let frame_interval = Duration::from_millis(33);

        loop {
            let loop_start = Instant::now();
            let mut frame = Mat::default();

            match cap.read_frame(&mut frame) {
                Ok(is_read) => {
                    // 프레임을 못 읽어오거나 비어있으면 루프를 종료합니다.
                    if !is_read || frame.empty() {
                        println!("[bsw] 비디오 스트림 종료.");
                        break;
                    }
                }
                Err(e) => {
                    eprintln!("[bsw] 에러: 프레임 읽기 실패: {:?}", e);
                    break;
                }
            }

            let cam_raw = Arc::new(DtoCamRaw::new(Arc::new(frame), frame_width, frame_height, alive_cnt));

            // 새로 만든 구조체의 소유권을 Arc로 넘깁니다.
            let _ = raw_tx.send(cam_raw.clone());

            alive_cnt += 1;

            let elapsed = loop_start.elapsed();
            if elapsed < frame_interval {
                thread::sleep(frame_interval - elapsed);
            }
        }

        Ok(())
    })
    .await
    .map_err(|e| opencv::Error::new(
        opencv::core::StsError,
        format!("Camera task join error: {}", e),
    ))?
}
