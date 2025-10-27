//! 신호등 인식 비전 태스크.
//! - 카메라 RAW 프레임을 HSV 공간으로 변환해 주 색상을 판별한다.
//! - 결과를 `DtoTrafficLight`로 만들어 RTE 채널에 게시한다.

use crate::asw::lib::vs_trafficlight_lib::*;
use crate::calibration::traffic_light::TrafficLightCalibration;
use crate::rte::rte_dto::DtoTrafficLight;
use crate::rte::rte_main::CameraChannels;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::RecvError;

/// 신호등 색상을 주기적으로 판별하는 러너블.
/// - 블로킹 캡처를 별도 스레드에서 수행하고, 결과만 비동기 채널로 보낸다.
/// - 일정 간격마다 감지하며, 최근 상태가 `Off`이면 즉시 재시도한다.
pub async fn runnable_vs_detect_trafficlight(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let traffic_calibration = TrafficLightCalibration::default();
    let detection_interval = traffic_calibration.detection_interval;
    let raw_tx = camera.raw_tx.clone();
    let traffic_tx = camera.traffic_light_tx.clone();
    // OpenCV 기반 처리이므로 블로킹 스레드에서 파이프라인을 실행한다.
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;
        let mut pipeline = Pipeline::new(traffic_calibration);
        let mut last_lag_log: Option<Instant> = None;
        let mut last_detected_color = TrafficLightColor::Off;

        loop {
            let mut cam_raw = match rx.blocking_recv() {
                Ok(cam_dto) => cam_dto, // 처리할 데이터만 추출
                Err(RecvError::Lagged(n)) => {
                    if last_lag_log
                        .map(|t| t.elapsed() > Duration::from_secs(1))
                        .unwrap_or(true)
                    {
                        eprintln!("[{}] Traffic light detector lagged by {}", id, n);
                        last_lag_log = Some(Instant::now());
                    }
                    continue;
                }
                Err(RecvError::Closed) => break,
            };

            // 최신 프레임만 처리하도록 버퍼를 비웁니다.
            while let Ok(newer) = rx.try_recv() {
                cam_raw = newer;
            }

            let should_detect = (alive_cnt % detection_interval == 0)
                || matches!(last_detected_color, TrafficLightColor::Off);
            if should_detect {
                // HSV 색 공간으로 변환한 뒤 색상 범위에 따라 신호등을 판별한다.
                let bgr_mat = cam_raw.as_bgr_mat()?;
                let hsv = pipeline.convert_to_hsv(&bgr_mat)?;
                let detected_color = pipeline.detect_color_from_hsv(&hsv);
                last_detected_color = detected_color;
            }

            // 결과 전송: 최신 감지 색상을 그대로 사용한다.
            let trafficlight_dto =
                Arc::new(DtoTrafficLight::new(last_detected_color.clone(), alive_cnt));
            let _ = traffic_tx.send(trafficlight_dto);

            alive_cnt += 1;
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Traffic light task join error: {}", e),
        )
    })?;

    join_result
}
