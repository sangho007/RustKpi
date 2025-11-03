//! ADAS(Advanced Driver Assistance System) 제어 모듈.
//! - `runnable_adas_lateral`: 차선 추정 결과를 이용해 조향 서보를 제어한다.
//! - `runnable_adas_longitudinal`: 장애물과 신호 정보를 사용해 종방향 속도를 결정한다.

use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{AdasLateralCalibration, AdasLongitudinalCalibration};
use crate::rte::rte_dto::{
    DtoCamLaneAngle, DtoDcMotorCtrl, DtoServoCtrl, DtoTrafficLight, DtoTrafficLightDirective,
    DtoUltraSonicObstacle, DtoUltraSonicRaw,
};
use crate::rte::rte_main::RteChannels;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::broadcast::error::TryRecvError;
use tokio::time;

/// ADAS Lateral 제어 러너블.
/// - 최신 차선 각도(laneAngle)를 읽어 비례 제어(P 제어)로 서보 목표각을 계산한다.
/// - 과도한 변화를 막기 위해 `max_servo_delta_deg`만큼 레이트 리밋을 적용한다.
/// - 결과를 `control.servo_tx` 채널로 퍼블리시한다.
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

        // 제어 주기 동기화
        tick.tick().await;

        // LaneAngle이 없다면 중립 유지
        let target_deg = if let Some(lane) = latest_lane.as_ref() {
            // 비례 제어: servo = neutral + k * angle
            let cmd = (calib.servo_neutral_deg as f64 + calib.lane_to_servo_gain * lane.angle)
                .round() as i32;
            cmd.clamp(calib.servo_min_deg as i32, calib.servo_max_deg as i32) as u32
        } else {
            // 데이터가 없으면 조향을 중립 상태로 유지한다.
            calib.servo_neutral_deg
        };

        // 레이트 리밋을 적용해 갑작스러운 조향 변화를 제한한다.
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

        // 명령 송신: DTO를 Arc로 감싸 브로드캐스트한다.
        let dto = DtoServoCtrl::new(calib.servo_channel_index, limited_deg);
        let _ = servo_tx.send(std::sync::Arc::new(dto));
        last_cmd_deg = limited_deg;

        // 1초마다 현재 제어 상태를 요약해 로깅한다.
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

/// ADAS Longitudinal 제어 러너블.
/// - 초음파 거리, 장애물 분류, 신호등 색상을 통합해 속도 명령을 생성한다.
/// - 멈춤/감속/순항 모드를 분기하고, 필요 시 Crawl 속도로 유지한다.
/// - 결과를 `control.dc_motor_tx` 채널로 브로드캐스트한다.
pub async fn runnable_adas_longitudinal(id: &'static str, channels: RteChannels) {
    let calib = AdasLongitudinalCalibration::default();

    let mut distance_rx = channels.ultrasonic.raw_tx.subscribe();
    let mut obstacle_rx = channels.ultrasonic.obstacle_tx.subscribe();
    let mut traffic_rx = channels.camera.traffic_light_tx.subscribe();
    let mut traffic_directive_rx = channels.camera.traffic_light_directive_tx.subscribe();
    let dc_tx = channels.control.dc_motor_tx.clone();

    let mut tick = time::interval(calib.control_period);

    // 가장 최근의 센싱 정보를 보관해 제어 주기마다 활용한다.
    let mut latest_distance: Option<DtoUltraSonicRaw> = None;
    let mut latest_obstacle: Option<DtoUltraSonicObstacle> = None;
    let mut latest_signal: Option<DtoTrafficLight> = None;
    let mut latest_directive: Option<DtoTrafficLightDirective> = None;
    let mut last_cmd: Option<(u32, u32)> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();

    loop {
        // 초음파 거리 측정값 확인
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

        // 장애물 판별 결과 확인
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

        // 신호등 인식 결과 확인
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
        match traffic_directive_rx.try_recv() {
            Ok(dto) => {
                latest_directive = Some(dto.as_ref().clone());
                while let Ok(newer) = traffic_directive_rx.try_recv() {
                    latest_directive = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!(
                    "[{}] ADAS longitudinal traffic directive lagged by {}",
                    id, n
                );
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        tick.tick().await;

        // 가장 최근 거리 값과 임계치 비교
        let distance_cm = latest_obstacle
            .as_ref()
            .map(|o| o.distance_cm)
            .or_else(|| latest_distance.as_ref().map(|d| d.distance));
        let distance_state = match distance_cm {
            Some(distance) => (
                distance <= calib.stop_distance_cm,
                distance <= calib.slowdown_distance_cm,
            ),
            None => (true, true),
        };
        let obstacle_status = latest_obstacle.as_ref();
        let stop_requested_obstacle = obstacle_status.map(|d| d.stop_requested).unwrap_or(false);
        let stop_requested_signal = latest_directive
            .as_ref()
            .map(|d| d.stop_requested && d.inside_detection_zone)
            .unwrap_or(false);
        let stop_requested = stop_requested_obstacle || stop_requested_signal;
        let lane_change_requested = obstacle_status
            .map(|d| d.lane_change_requested)
            .unwrap_or(false);
        let accelerate_requested = latest_directive
            .as_ref()
            .map(|d| d.accelerate_requested && d.inside_detection_zone)
            .unwrap_or(false);
        let traffic_color = latest_signal
            .as_ref()
            .map(|signal| signal.traffic_light_color.clone());

        // 장애물·신호·거리 조건을 종합해 정지 여부를 결정한다.
        let need_stop = stop_requested
            || matches!(traffic_color.as_ref(), Some(TrafficLightColor::Red))
            || distance_state.0;

        let caution_signal = matches!(
            traffic_color.as_ref(),
            Some(TrafficLightColor::Yellow | TrafficLightColor::Off)
        );
        let need_caution = if accelerate_requested {
            false
        } else {
            caution_signal || distance_state.1 || lane_change_requested
        };

        let (direction, speed) = if need_stop {
            (0, 0)
        } else if need_caution {
            (1, calib.crawl_speed_percent)
        } else {
            (1, calib.cruise_speed_percent)
        };

        let command = (direction, speed);
        if last_cmd.map(|prev| prev != command).unwrap_or(true) {
            // 명령이 변경되었을 때만 DC 모터 채널로 전송해 불필요한 통신을 줄인다.
            let dto = DtoDcMotorCtrl::new(direction, speed, alive_cnt);
            let _ = dc_tx.send(Arc::new(dto));
            alive_cnt = alive_cnt.wrapping_add(1);
            last_cmd = Some(command);
        }

        // 설정된 로깅 주기에 따라 상태를 출력한다.
        if last_log.elapsed() >= calib.log_interval {
            let distance_str = distance_cm
                .map(|d| format!("{:.1}", d))
                .unwrap_or_else(|| "--".to_string());
            println!(
                "[{}] Longitudinal: dist={}cm stop_req={} lane_change={} accel_req={} signal={:?} -> dir={} speed={}",
                id,
                distance_str,
                stop_requested,
                lane_change_requested,
                accelerate_requested,
                traffic_color,
                direction,
                speed
            );
            last_log = Instant::now();
        }
    }
}
