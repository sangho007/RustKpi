//! BSW 카메라 ECU 모듈.
//! - 실제 차량에서는 libcamera 브릿지를 통해 센서에서 직접 프레임을 수집한다.
//! - 개발 환경에서는 샘플 동영상을 이용해 동일한 인터페이스로 프레임을 공급한다.
//! - 수집된 영상은 RTE 카메라 채널에 Raw DTO 형태로 공유된다.

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

/// 카메라 캡처 스레드를 실행하고 프레임을 RTE RAW 채널로 전송한다.
/// - std::thread로 블로킹 캡처 루프를 실행하며, Tokio 채널로 비동기 영역과 연결한다.
/// - 채널 수신자가 모두 떠나면 반복문을 종료해 스레드를 정리한다.
pub async fn ea_cam_provider(camera: CameraChannels) -> Result<()> {
    let camera_calibration = CameraCalibration::default();
    let CameraChannels { raw_tx, .. } = camera;
    // 캡처 스레드와 비동기 태스크 사이의 완충 버퍼. 설정값에 따라 큐 길이가 결정된다.
    let (frame_tx, mut frame_rx) =
        mpsc::channel::<cam_lib::CapturedFrame>(camera_calibration.capture_queue_depth);

    let capture_config = camera_calibration;
    // OpenCV 또는 libcamera API는 OS 스레드를 요구하므로 std::thread로 분리한다.
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

        // 다운스트림 소비자가 빠르게 변환할 수 있도록 Arc로 공유한다.
        let _ = raw_tx.send(cam_raw);
        alive_cnt = alive_cnt.wrapping_add(1);
    }

    // 블로킹 join 연산을 비동기 런타임에 안전하게 위임한다.
    let join_result = task::spawn_blocking(move || capture_thread.join())
        .await
        .map_err(|e| {
            opencv::Error::new(
                opencv::core::StsError,
                format!("Camera capture join task failed: {}", e),
            )
        })?;

    // 캡처 스레드가 정상 종료되었는지 판별한다.
    match join_result {
        Ok(Ok(())) => Ok(()),
        Ok(Err(err)) => Err(err),
        Err(_) => Err(opencv::Error::new(
            opencv::core::StsError,
            "Camera capture thread panicked",
        )),
    }
}

/// 카메라 장치를 지속적으로 읽는 블로킹 루프.
/// - 선택한 백엔드(libcamera 또는 비디오 파일)를 열고 프레임을 반복적으로 읽는다.
/// - 읽기에 실패하면 재시도하며, 채널이 닫히면 즉시 종료한다.
fn camera_capture_loop(
    frame_tx: mpsc::Sender<cam_lib::CapturedFrame>,
    camera_calibration: CameraCalibration,
) -> Result<()> {
    // 목표 FPS에 맞춰 프레임을 균일하게 공급하기 위해 다음 프레임 예정 시각을 추적한다.
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
                    // 타이밍 보정을 통해 일정한 간격으로 프레임을 공급한다.
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
                    // EOF 또는 일시적 공백 프레임. 백엔드를 다시 초기화해본다.
                    println!("[bsw] 비디오 스트림 종료. 재시도합니다.");
                    break;
                }
                Err(e) => {
                    // 장치 오류 또는 디코딩 오류. 재초기화 후 재시도한다.
                    eprintln!("[bsw] 프레임 읽기 실패: {e:?}. 캡처를 재시도합니다.");
                    break;
                }
            }
        }

        if frame_tx.is_closed() {
            return Ok(());
        }

        // 재시도까지 약간의 지연을 두어 장치 초기화 시간을 벌어준다.
        thread::sleep(Duration::from_millis(500));
    }
}

/// 사용할 캡처 백엔드를 결정하고 초기화한다.
/// - 실제 모드에서는 libcamera 브릿지를 통해 카메라 센서를 연다.
/// - 개발 모드에서는 샘플 비디오 파일을 열어 동일한 인터페이스를 제공한다.
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
        // 현장 녹화 파일이 존재하면 우선적으로 사용하고, 없으면 폴백 파일을 연다.
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
