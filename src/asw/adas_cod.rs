use crate::calibration::AdasLateralCalibration;
use crate::rte::rte_dto::{DtoCamLaneAngle, DtoServoCtrl};
use crate::rte::rte_main::RteChannels;
use std::time::Instant;
use tokio::sync::broadcast::error::TryRecvError;
use tokio::time;

/// ADAS Lateral 제어 러너블: 차선 각도(laneAngle)에 비례하여 서보 각도를 산출한다.
///
/// - 입력: `camera.lane_angle_tx`
/// - 출력: `control.servo_tx` (`DtoServoCtrl`)
pub async fn runnable_adas_lateral(id: &'static str, channels: RteChannels) {
    let calib = AdasLateralCalibration::default();
    let mut lane_rx = channels.camera.lane_angle_tx.subscribe();
    let servo_tx = channels.control.servo_tx.clone();

    // 제어 루프 주기(기본 50ms)
    let mut tick = time::interval(std::time::Duration::from_millis(50));

    // 최신 신호 캐시
    let mut latest_lane: Option<DtoCamLaneAngle> = None;
    let mut last_cmd_deg: u32 = calib.servo_neutral_deg;
    let mut last_log: Instant = Instant::now();

    loop {
        // 새 메시지가 도착했으면 최신으로 드레인
        match lane_rx.try_recv() {
            Ok(dto) => {
                latest_lane = Some(dto.as_ref().clone());
                while let Ok(newer) = lane_rx.try_recv() {
                    latest_lane = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS lateral lane_angle lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {
                // Closed는 다음 루프에서 publish 없이 진행; Empty는 무시
            }
        }

        tick.tick().await;

        // LaneAngle이 없다면 중립 유지
        let target_deg = if let Some(lane) = latest_lane.as_ref() {
            // 비례 제어: servo = neutral + k * angle
            let cmd = (calib.servo_neutral_deg as f64
                + calib.lane_to_servo_gain * lane.angle)
                .round() as i32;
            cmd.clamp(calib.servo_min_deg as i32, calib.servo_max_deg as i32) as u32
        } else {
            calib.servo_neutral_deg
        };

        // 레이트 리밋 적용
        let delta = if target_deg >= last_cmd_deg {
            target_deg - last_cmd_deg
        } else {
            last_cmd_deg - target_deg
        };
        let limited_deg = if delta > calib.max_servo_delta_deg {
            if target_deg > last_cmd_deg {
                last_cmd_deg + calib.max_servo_delta_deg
            } else {
                last_cmd_deg - calib.max_servo_delta_deg
            }
        } else {
            target_deg
        };

        // 명령 송신
        let dto = DtoServoCtrl::new(calib.servo_channel_index, limited_deg);
        let _ = servo_tx.send(std::sync::Arc::new(dto));
        last_cmd_deg = limited_deg;

        if last_log.elapsed() > std::time::Duration::from_secs(1) {
            if let Some(lane) = latest_lane.as_ref() {
                println!(
                    "[{}] Lateral: lane_angle={:.2} -> servo={}deg",
                    id, lane.angle, last_cmd_deg
                );
            } else {
                println!(
                    "[{}] Lateral: lane_angle=-- -> servo={}deg",
                    id, last_cmd_deg
                );
            }
            last_log = Instant::now();
        }
    }
}
