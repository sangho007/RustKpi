//! 초음파 기반 전방 충돌 감지 러너블.
//! - 초음파 RAW 거리를 읽어 지정 임계값 이하이면 장애물로 판단한다.
//! - 판단 결과를 RTE `obstacle_tx` 채널에 게시해 종방향 제어가 사용할 수 있도록 한다.

use crate::asw::lib::forwardcollision_ultrasonic_lib::forward_collision_calibration;
use crate::rte::rte_dto::DtoUltraSonicObstacle;
use crate::rte::rte_main::UltrasonicChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

/// 초음파 거리 값을 임계치와 비교해 장애물 감지를 수행한다.
/// - 계산은 CPU 바운드가 아니므로 블로킹 스레드에서 처리한다.
/// - 결과는 `DtoUltraSonicObstacle` DTO로 변환해 브로드캐스트한다.
pub async fn runnable_forwardcollision_obstacle_detection(
    id: &'static str,
    channels: UltrasonicChannels,
) {
    let calibration = forward_collision_calibration();
    let raw_tx = channels.raw_tx.clone();
    let obstacle_tx = channels.obstacle_tx.clone();
    // 블로킹 수신을 별도의 스레드에서 처리해 Tokio 런타임을 막지 않는다.
    let handle = tokio::task::spawn_blocking(move || {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;
        let mut stop_cnt = 0;
        let mut change_req_cnt = 0;
        let stop_threshold = calibration.stop_request_distance_cm;
        let lane_change_threshold = calibration
            .lane_change_request_distance_cm
            .max(stop_threshold);

        loop {
            match rx.blocking_recv() {
                Ok(ultrasonic_dto) => {
                    // 거리(cm)에 따라 정지/차선 변경 요청 여부를 판정한다.
                    let distance = ultrasonic_dto.distance;
                    if (distance <= stop_threshold) {
                        stop_cnt += 1;
                    }
                    if (distance <= lane_change_threshold) {
                        change_req_cnt += 1;
                    }

                    let mut stop_requested = false;
                    let mut lane_change_requested = false;

                    if (stop_cnt >= 3) {
                        stop_requested = true;
                    }else {
                        stop_cnt = 0;
                        stop_requested = false;
                    }

                    if (change_req_cnt >= 3) {
                        lane_change_requested = true;
                    }else {
                        change_req_cnt = 0;
                        lane_change_requested = true;
                    }


                    //let stop_requested = distance <= stop_threshold;
                    //let lane_change_requested = distance <= lane_change_threshold;
                    let obstacle_dto = Arc::new(DtoUltraSonicObstacle::new(
                        stop_requested,
                        lane_change_requested,
                        distance,
                        alive_cnt,
                    ));
                    let _ = obstacle_tx.send(obstacle_dto);

                    alive_cnt += 1;
                }
                Err(RecvError::Lagged(n)) => {
                    // 소비자가 늦게 따라오면 경고만 남기고 계속 진행한다.
                    eprintln!("[{}] Ultrasound obstacle detector lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            }
        }
    });

    // 블로킹 태스크가 패닉했는지 확인한다.
    if let Err(e) = handle.await {
        eprintln!("[{}] Ultrasound obstacle detector join error: {}", id, e);
    }
}
