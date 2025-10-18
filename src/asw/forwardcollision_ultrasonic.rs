use crate::asw::lib::forwardcollision_ultrasonic_lib::*;
use crate::rte::rte_dto::DtoUltraSonicObstacle;
use crate::rte::rte_main::UltrasonicChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_obstacle_detection(id: &'static str, channels: UltrasonicChannels) {
    let raw_tx = channels.raw_tx.clone();
    let obstacle_tx = channels.obstacle_tx.clone();
    let handle = tokio::task::spawn_blocking(move || {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;

        loop {
            match rx.blocking_recv() {
                Ok(ultrasonic_dto) => {
                    let detected = ultrasonic_dto.distance < THRESHOLD_DISTANCE;
                    let obstacle_dto = Arc::new(DtoUltraSonicObstacle::new(detected, alive_cnt));
                    let _ = obstacle_tx.send(obstacle_dto);

                    alive_cnt += 1;
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[{}] Ultrasound obstacle detector lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            }
        }
    });

    if let Err(e) = handle.await {
        eprintln!("[{}] Ultrasound obstacle detector join error: {}", id, e);
    }
}
