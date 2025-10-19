use crate::bsw::lib::cam_lib;
use crate::rte::rte_dto::DtoCamRaw;
use crate::rte::rte_main::CameraChannels;
use opencv::{Result, videoio};
use std::sync::Arc;

pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    tokio::task::spawn_blocking(move || -> Result<()> {
        let CameraChannels { raw_tx, .. } = camera;
        let mut alive_cnt = 0;

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
            // videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?
            let file_cap =
                videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?;
            Box::new(file_cap)
        };

        loop {
            match cap.read_frame() {
                Ok(Some(captured)) => {
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

                    alive_cnt += 1;
                }
                Ok(None) => {
                    println!("[bsw] 비디오 스트림 종료.");
                    break;
                }
                Err(e) => {
                    eprintln!("[bsw] 에러: 프레임 읽기 실패: {:?}", e);
                    break;
                }
            }
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Camera task join error: {}", e),
        )
    })?
}
