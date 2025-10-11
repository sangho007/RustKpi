use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;
use crate::asw::lib::uss_lib::*;
use crate::rte::rte_main::{DebugSender, VfbSender};
use crate::rte::rte_dto::{VfbEvent,DtoUltraSonicObstacle};

pub async fn runnable_obstacle_detection(id: &'static str, tx: VfbSender, debug: DebugSender) {
    let mut rx = tx.subscribe();
    let mut alive_cnt = 0;
    let mut detected;
    let mut obstacle_dto;
    let mut event;

    loop {
        match rx.recv().await {
            Ok(VfbEvent::UltraSonicRawEvent(ultrasonic_dto)) => {
                if ultrasonic_dto.distance < THRESHOLD_DISTANCE {detected = true;}
                else {detected = false;}

                obstacle_dto = DtoUltraSonicObstacle::new(detected,alive_cnt);
                event = VfbEvent::UltraSonicObstacleDetectedEvent(Arc::new(obstacle_dto));
                let _ = tx.send(event.clone());
                let _ = debug.send(event.clone());

                alive_cnt += 1;
            }
            Err(RecvError::Lagged(n)) => { continue; }
            _=> continue,
        }
    }
}