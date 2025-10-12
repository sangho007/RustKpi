use crate::asw::lib::vs_trafficlight_lib::*;
use crate::rte::rte_dto::{DtoTrafficLight, VfbEvent};
use crate::rte::rte_main::{DebugSender, VfbSender};
use opencv::core::Mat;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;


pub async fn runnable_trafficlight_detection(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<Mat> {
    let mut rx = tx.subscribe();
    let mut alive_cnt = 0;
    let mut trafficlight_dto;
    let mut hsv;
    let mut detected_color;
    let mut event;

    let mut pipeline = Pipeline::new();

    loop {
        let cam_raw = match rx.recv().await {
            Ok(VfbEvent::CamRawEvent(cam_dto)) => cam_dto, // 처리할 데이터만 추출
            Err(RecvError::Lagged(n)) => { continue; }
            _ => { continue; } // 관심 없는 이벤트는 무시
        };

        hsv = pipeline.convert_to_hsv(&cam_raw.img)?;
        detected_color = pipeline.detect_color_from_hsv(&hsv);

        // 3. 결과 전송
        trafficlight_dto = DtoTrafficLight::new(detected_color, alive_cnt);
        event = VfbEvent::CamTrafficLightEvent(Arc::new(trafficlight_dto));

        let _ = tx.send(event.clone());
        let _ = debug.send(event.clone());

        alive_cnt += 1;
    }
}