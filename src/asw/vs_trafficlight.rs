use crate::asw::lib::vs_trafficlight_lib::*;
use crate::rte::rte_dto::DtoTrafficLight;
use crate::rte::rte_main::CameraChannels;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_trafficlight_detection(
    id: &'static str,
    camera: CameraChannels,
) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let traffic_tx = camera.traffic_light_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;
        let mut pipeline = Pipeline::new();
        let mut last_lag_log: Option<Instant> = None;

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

            let hsv = pipeline.convert_to_hsv(&cam_raw.img)?;
            let detected_color = pipeline.detect_color_from_hsv(&hsv);

            // 3. 결과 전송
            let trafficlight_dto = Arc::new(DtoTrafficLight::new(detected_color, alive_cnt));
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
