//! ADAS(Advanced Driver Assistance System) 제어 모듈.
//! - `runnable_adas_lateral`: 차선 추정 결과를 이용해 조향 서보를 제어한다.
//! - `runnable_adas_longitudinal`: 장애물과 신호 정보를 사용해 종방향 속도를 결정한다.

use crate::asw::lib::adas_path_lib::curvature_from_smoothed_path;
use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{AdasLateralCalibration, AdasLongitudinalCalibration};
use crate::rte::rte_dto::{
    AdasLaneChangeState, DtoAdasSmoothedPath, DtoCamLaneAngle, DtoDcMotorCtrl, DtoServoCtrl,
    DtoTrafficLight, DtoTrafficLightDirective, DtoUltraSonicObstacle, DtoUltraSonicRaw,
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
    let mut path_rx = channels.path.smoothed_tx.subscribe();
    let servo_tx = channels.control.servo_tx.clone();

    // 제어 루프 주기(기본 50ms)
    let mut tick = time::interval(std::time::Duration::from_millis(50));

    // 최신 신호 캐시
    let mut latest_lane: Option<DtoCamLaneAngle> = None;
    let mut latest_path: Option<DtoAdasSmoothedPath> = None;
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

        match path_rx.try_recv() {
            Ok(dto) => {
                latest_path = Some(dto.as_ref().clone());
                while let Ok(newer) = path_rx.try_recv() {
                    latest_path = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS lateral smoothed_path lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        // 제어 주기 동기화
        tick.tick().await;

        let lane_state = latest_path.as_ref().map(|path| path.lane_change_state);

        let curvature = if let Some(path) = latest_path.as_ref() {
            match curvature_from_smoothed_path(path) {
                Ok(value) => value,
                Err(err) => {
                    eprintln!("[{}] ADAS lateral curvature 계산 실패: {}", id, err);
                    0.0
                }
            }
        } else {
            0.0
        };

        let mut lane_offset = latest_lane
            .as_ref()
            .map(|lane| lane.lateral_offset)
            .unwrap_or(0.0);
        let mut lane_angle = latest_lane.as_ref().map(|lane| lane.angle).unwrap_or(0.0);
        if let Some(state) = lane_state {
            if !matches!(
                state,
                AdasLaneChangeState::InnerCruise | AdasLaneChangeState::OuterCruise
            ) {
                lane_offset = 0.0;
                lane_angle = 0.0;
            }
        }

        let target_angle = calib.curvature_to_servo_gain * curvature
            + calib.lane_to_servo_gain * lane_angle
            + calib.lateral_offset_gain * lane_offset;

        let base_cmd = calib.servo_neutral_deg as f64 + target_angle;
        let target_deg = base_cmd
            .round()
            .clamp(calib.servo_min_deg as f64, calib.servo_max_deg as f64)
            as u32;

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
            let lane_angle_display = latest_lane
                .as_ref()
                .map(|lane| format!("{:.2}", lane.angle))
                .unwrap_or_else(|| "--".to_string());
            let lane_offset_display = latest_lane
                .as_ref()
                .map(|lane| format!("{:.2}", lane.lateral_offset))
                .unwrap_or_else(|| "--".to_string());
            let state_display = lane_state
                .map(|state| format!("{:?}", state))
                .unwrap_or_else(|| "--".to_string());
            println!(
                "[{}] Lateral: curvature={:.4}, lane_angle={} offset={} state={} -> servo={}deg",
                id, curvature, lane_angle_display, lane_offset_display, state_display, last_cmd_deg
            );
            last_log = Instant::now();
        }
    }
}

fn apply_speed_rate_limit(
    previous_speed: u32,
    desired_speed: u32,
    calib: &AdasLongitudinalCalibration,
) -> u32 {
    if desired_speed > previous_speed {
        let step = calib.max_accel_delta_percent.max(1);
        let delta = desired_speed - previous_speed;
        previous_speed + delta.min(step)
    } else if desired_speed < previous_speed {
        let step = calib.max_decel_delta_percent.max(1);
        let delta = previous_speed - desired_speed;
        previous_speed.saturating_sub(delta.min(step))
    } else {
        desired_speed
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
    let mut path_rx = channels.path.smoothed_tx.subscribe();
    let dc_tx = channels.control.dc_motor_tx.clone();

    let mut tick = time::interval(calib.control_period);

    // 가장 최근의 센싱 정보를 보관해 제어 주기마다 활용한다.
    let mut latest_distance: Option<DtoUltraSonicRaw> = None;
    let mut latest_obstacle: Option<DtoUltraSonicObstacle> = None;
    let mut latest_signal: Option<DtoTrafficLight> = None;
    let mut latest_directive: Option<DtoTrafficLightDirective> = None;
    let mut latest_path: Option<DtoAdasSmoothedPath> = None;
    let mut last_cmd: Option<(u32, u32)> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();
    let mut stop_request_since: Option<Instant> = None;
    let mut stop_release_since: Option<Instant> = None;
    let mut stop_engaged = false;

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
        match path_rx.try_recv() {
            Ok(dto) => {
                latest_path = Some(dto.as_ref().clone());
                while let Ok(newer) = path_rx.try_recv() {
                    latest_path = Some(newer.as_ref().clone());
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal smoothed_path lagged by {}", id, n);
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
        let obstacle_stop = stop_requested_obstacle;
        let traffic_stop =
            matches!(traffic_color.as_ref(), Some(TrafficLightColor::Red)) || stop_requested_signal;

        let caution_signal = matches!(
            traffic_color.as_ref(),
            Some(TrafficLightColor::Yellow | TrafficLightColor::Off)
        );
        let need_caution = if accelerate_requested {
            false
        } else {
            caution_signal || distance_state.1 || lane_change_requested
        };

        let path_ready = latest_path
            .as_ref()
            .map(|path| !path.samples_xy.is_empty())
            .unwrap_or(false);
        let gating_stop = !path_ready;

        let curvature_abs = latest_path
            .as_ref()
            .and_then(|path| curvature_from_smoothed_path(path).ok())
            .map(|curv| curv.abs())
            .unwrap_or(0.0);

        let mut gain = 1.0;
        if curvature_abs > calib.curvature_slowdown_threshold {
            gain *= 0.8;
        }

        let base_stop_reason = if obstacle_stop {
            Some("obstacle")
        } else if traffic_stop {
            Some("traffic")
        } else {
            None
        };
        let mut effective_stop_reason = base_stop_reason;
        if gating_stop {
            effective_stop_reason = Some("no_path");
        }

        let now = Instant::now();
        if let Some(_reason) = effective_stop_reason {
            if stop_request_since.is_none() {
                stop_request_since = Some(now);
            }
            stop_release_since = None;
            if stop_request_since
                .map(|t| now.duration_since(t) >= calib.stop_request_hold_time)
                .unwrap_or(false)
            {
                stop_engaged = true;
            }
        } else {
            stop_request_since = None;
            if stop_engaged {
                if stop_release_since.is_none() {
                    stop_release_since = Some(now);
                }
                if stop_release_since
                    .map(|t| now.duration_since(t) >= calib.stop_release_hold_time)
                    .unwrap_or(false)
                {
                    stop_engaged = false;
                    stop_release_since = None;
                }
            }
        }

        if stop_engaged || gating_stop {
            gain = 0.0;
        } else if need_caution {
            gain *= 0.6;
        }

        let mut desired_command = if gain <= 0.0 {
            (0, 0)
        } else {
            let commanded = (calib.cruise_speed_percent as f64 * gain)
                .round()
                .clamp(0.0, calib.cruise_speed_percent as f64) as u32;
            if commanded == 0 {
                (0, 0)
            } else {
                (1, commanded)
            }
        };
        if gating_stop {
            desired_command = (0, 0);
        }

        let previous_speed = last_cmd.map(|(_, spd)| spd).unwrap_or(0);
        let limited_speed = apply_speed_rate_limit(previous_speed, desired_command.1, &calib);
        let limited_direction = if limited_speed == 0 { 0 } else { 1 };
        let command = (limited_direction, limited_speed);

        if last_cmd.map(|prev| prev != command).unwrap_or(true) {
            // 명령이 변경되었을 때만 DC 모터 채널로 전송해 불필요한 통신을 줄인다.
            let dto = DtoDcMotorCtrl::new(command.0, command.1, alive_cnt);
            let _ = dc_tx.send(Arc::new(dto));
            alive_cnt = alive_cnt.wrapping_add(1);
            last_cmd = Some(command);
        }

        // 설정된 로깅 주기에 따라 상태를 출력한다.
        if last_log.elapsed() >= calib.log_interval {
            let distance_str = distance_cm
                .map(|d| format!("{:.1}", d))
                .unwrap_or_else(|| "--".to_string());
            let stop_display = if stop_engaged || gating_stop {
                effective_stop_reason.unwrap_or("--")
            } else {
                "--"
            };
            println!(
                "[{}] Longitudinal: dist={}cm curvature={:.4} gain={:.2} stop={} lane_change={} accel_req={} path_ready={} signal={:?} -> dir={} speed={}",
                id,
                distance_str,
                curvature_abs,
                gain,
                stop_display,
                lane_change_requested,
                accelerate_requested,
                path_ready,
                traffic_color,
                command.0,
                command.1
            );
            last_log = Instant::now();
        }
    }
}
