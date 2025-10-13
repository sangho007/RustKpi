use crate::bsw::lib::cam_lib;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::core::Mat;
use opencv::{prelude::*, videoio, Result};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    tokio::task::spawn_blocking(move || -> Result<()> {
        let CameraChannels { raw_tx, .. } = camera;
        let mut alive_cnt = 0;
        
        // 비디오 파일이나 카메라 장치를 엽니다.
        let cammode = false;
        let mut cap: Box<dyn cam_lib::FrameCapture> = if cammode {
            // videoio::VideoCapture::new(0, videoio::CAP_ANY)?
            let picam = cam_lib::picamera_capture::PiCamera2::new(640, 480)
                // 3. Map the Python error to an OpenCV error to unify error types.
                .map_err(|e| opencv::Error::new(opencv::core::StsError, format!("Failed to initialize PiCamera2: {}", e)))?;
            Box::new(picam)
        } else {
            // videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?
            let file_cap = videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?;
            Box::new(file_cap)
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

            let cam_raw = Arc::new(DtoCamRaw::new(Arc::new(frame), 1280, 720, alive_cnt));

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
