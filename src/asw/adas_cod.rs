use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{AdasLateralCalibration, AdasLongitudinalCalibration};
use crate::rte::rte_dto::{
    DtoCamLaneAngle, DtoDcMotorCtrl, DtoServoCtrl, DtoTrafficLight, DtoUltraSonicObstacle,
    DtoUltraSonicRaw,
};
use crate::rte::rte_main::RteChannels;
use std::sync::Arc;
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
            let cmd = (calib.servo_neutral_deg as f64 + calib.lane_to_servo_gain * lane.angle)
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

/// ADAS Longitudinal 제어 러너블: 장애물·신호등 정보를 기반으로 DC 모터 속도를 결정한다.
///
/// - 입력: `ultrasonic.raw_tx`, `ultrasonic.obstacle_tx`, `camera.traffic_light_tx`
/// - 출력: `control.dc_motor_tx` (`DtoDcMotorCtrl`)
pub async fn runnable_adas_longitudinal(id: &'static str, channels: RteChannels) {
    let calib = AdasLongitudinalCalibration::default();

    let mut distance_rx = channels.ultrasonic.raw_tx.subscribe();
    let mut obstacle_rx = channels.ultrasonic.obstacle_tx.subscribe();
    let mut traffic_rx = channels.camera.traffic_light_tx.subscribe();
    let dc_tx = channels.control.dc_motor_tx.clone();

    let mut tick = time::interval(calib.control_period);

    let mut latest_distance: Option<DtoUltraSonicRaw> = None;
    let mut latest_obstacle: Option<DtoUltraSonicObstacle> = None;
    let mut latest_signal: Option<DtoTrafficLight> = None;
    let mut last_cmd: Option<(u32, u32)> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();

    loop {
        match distance_rx.try_recv() {
            Ok(dto) => {
                latest_distance = Some(dto.as_ref().clone());
                while let Ok(newer) = distance_rx.try_recv() {
                    latest_distance = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal distance lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        match obstacle_rx.try_recv() {
            Ok(dto) => {
                latest_obstacle = Some(dto.as_ref().clone());
                while let Ok(newer) = obstacle_rx.try_recv() {
                    latest_obstacle = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal obstacle lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        match traffic_rx.try_recv() {
            Ok(dto) => {
                latest_signal = Some(dto.as_ref().clone());
                while let Ok(newer) = traffic_rx.try_recv() {
                    latest_signal = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal traffic lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        tick.tick().await;

        let distance_cm = latest_distance.as_ref().map(|d| d.distance);
        let distance_state = match distance_cm {
            Some(distance) => (
                distance <= calib.stop_distance_cm,
                distance <= calib.slowdown_distance_cm,
            ),
            None => (true, true),
        };
        let obstacle_detected = latest_obstacle
            .as_ref()
            .map(|d| d.detected)
            .unwrap_or(false);
        let traffic_color = latest_signal
            .as_ref()
            .map(|signal| signal.traffic_light_color.clone());

        let need_stop = obstacle_detected
            || matches!(traffic_color.as_ref(), Some(TrafficLightColor::Red))
            || distance_state.0;

        let caution_signal = matches!(
            traffic_color.as_ref(),
            Some(TrafficLightColor::Yellow | TrafficLightColor::Off)
        );
        let need_caution = caution_signal || distance_state.1;

        let (direction, speed) = if need_stop {
            (0, 0)
        } else if need_caution {
            (1, calib.crawl_speed_percent)
        } else {
            (1, calib.cruise_speed_percent)
        };

        let command = (direction, speed);
        if last_cmd.map(|prev| prev != command).unwrap_or(true) {
            let dto = DtoDcMotorCtrl::new(direction, speed, alive_cnt);
            let _ = dc_tx.send(Arc::new(dto));
            alive_cnt = alive_cnt.wrapping_add(1);
            last_cmd = Some(command);
        }

        if last_log.elapsed() >= calib.log_interval {
            let distance_str = distance_cm
                .map(|d| format!("{:.1}", d))
                .unwrap_or_else(|| "--".to_string());
            println!(
                "[{}] Longitudinal: dist={}cm obstacle={} signal={:?} -> dir={} speed={}",
                id, distance_str, obstacle_detected, traffic_color, direction, speed
            );
            last_log = Instant::now();
        }
    }
}
