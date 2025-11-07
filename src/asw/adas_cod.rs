//! ADAS(Advanced Driver Assistance System) 제어 모듈.
//! - `runnable_adas_lateral`: 차선 추정 결과를 이용해 조향 서보를 제어한다.
//! - `runnable_adas_longitudinal`: 장애물과 신호 정보를 사용해 종방향 속도를 결정한다.

use crate::asw::lib::adas_path_lib::curvature_from_smoothed_path;
use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{AdasLateralCalibration, AdasLongitudinalCalibration};
use crate::rte::rte_dto::{
    AdasLaneChangeState, DtoAdasSmoothedPath, DtoCamLaneAngle, DtoDcMotorCtrl,
    DtoLocalizationArrival, DtoLocalizationState, DtoServoCtrl, DtoTrafficLight,
    DtoTrafficLightDirective, DtoUltraSonicObstacle, DtoUltraSonicRaw,
};
use crate::rte::rte_main::RteChannels;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::TryRecvError;
use tokio::time;

/// ADAS Lateral 제어 러너블.
/// - 차량 현재 yaw로 정의한 직선 참조 축을 기준으로 스무딩 경로의 횡오차를 계산한다.
/// - PID 제어기를 통해 서보 목표각을 계산하고, `max_servo_delta_deg` 레이트 리밋을 적용한다.
/// - 결과를 `control.servo_tx` 채널로 퍼블리시한다.
pub async fn runnable_adas_lateral(id: &'static str, channels: RteChannels) {
    let calib = AdasLateralCalibration::from_env();
    let mut path_rx = channels.path.smoothed_tx.subscribe();
    let mut localization_rx = channels.localization.state_tx.subscribe();
    let mut lane_angle_rx = channels.camera.lane_angle_tx.subscribe();
    let servo_tx = channels.control.servo_tx.clone();

    // 제어 루프 주기(기본 50ms)
    let control_period = Duration::from_millis(50);
    let mut tick = time::interval(control_period);
    let dt_sec = control_period.as_secs_f64();

    // 최신 신호 캐시
    let mut latest_path: Option<Arc<DtoAdasSmoothedPath>> = None;
    let mut latest_state: Option<Arc<DtoLocalizationState>> = None;
    let mut latest_lane_angle: Option<Arc<DtoCamLaneAngle>> = None;
    let mut last_cmd_deg: u32 = calib.servo_neutral_deg;
    let mut last_log: Instant = Instant::now();
    let mut integral_error: f64 = 0.0;
    let mut prev_error: Option<f64> = None;

    loop {
        // 새 메시지가 도착했으면 최신으로 드레인
        match localization_rx.try_recv() {
            Ok(dto) => {
                let mut newest = dto;
                while let Ok(newer) = localization_rx.try_recv() {
                    newest = newer;
                }
                latest_state = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS lateral localization lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {
                // Closed는 다음 루프에서 publish 없이 진행; Empty는 무시
            }
        }

        match path_rx.try_recv() {
            Ok(dto) => {
                let mut newest = dto;
                while let Ok(newer) = path_rx.try_recv() {
                    newest = newer;
                }
                latest_path = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS lateral smoothed_path lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        match lane_angle_rx.try_recv() {
            Ok(dto) => {
                let mut newest = dto;
                while let Ok(newer) = lane_angle_rx.try_recv() {
                    newest = newer;
                }
                latest_lane_angle = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS lateral lane_angle lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        // 제어 주기 동기화
        tick.tick().await;

        let lane_state = latest_path.as_deref().map(|path| path.lane_change_state);

        let mut pid_output = 0.0;
        let mut current_error = None;

        if let (Some(state), Some(smoothed_path)) =
            (latest_state.as_deref(), latest_path.as_deref())
        {
            current_error = compute_lateral_error(state, smoothed_path, calib.pid_sample_index);
        }

        if let Some(state) = lane_state {
            if !matches!(
                state,
                AdasLaneChangeState::InnerCruise | AdasLaneChangeState::OuterCruise
            ) {
                integral_error = 0.0;
                prev_error = None;
                current_error = Some(0.0);
            }
        }

        if let Some(error) = current_error {
            integral_error += error * dt_sec;
            if calib.pid_integral_limit > 0.0 {
                let limit = calib.pid_integral_limit;
                integral_error = integral_error.clamp(-limit, limit);
            }
            let derivative = if let Some(prev) = prev_error {
                (error - prev) / dt_sec
            } else {
                0.0
            };
            prev_error = Some(error);
            pid_output =
                calib.pid_kp * error + calib.pid_ki * integral_error + calib.pid_kd * derivative;
        } else {
            // 데이터 부족 시 PID 상태를 리셋해 드리프트를 방지한다.
            integral_error = 0.0;
            prev_error = None;
        }

        let lane_offset_px = latest_lane_angle
            .as_ref()
            .map(|lane| lane.lateral_offset)
            .unwrap_or(0.0);

        //let base_cmd = calib.servo_neutral_deg as f64 - pid_output; -> good !!!!!
        let base_cmd = calib.servo_neutral_deg as f64
            - 10.0 * current_error.unwrap_or(0.0);
            //+ 0.05 * lane_offset_px;

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
            let error_display = current_error
                .map(|err| format!("{:.3}", err))
                .unwrap_or_else(|| "--".to_string());
            let state_display = lane_state
                .map(|state| format!("{:?}", state))
                .unwrap_or_else(|| "--".to_string());
            let yaw_display = latest_state
                .as_deref()
                .map(|s| format!("{:.2}", s.yaw_rad))
                .unwrap_or_else(|| "--".to_string());
            println!(
                "[{}] Lateral PID: error={} integ={:.3} yaw={} state={} -> servo={}deg",
                id, error_display, integral_error, yaw_display, state_display, last_cmd_deg
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
    let mut arrival_rx = channels.localization.arrival_tx.subscribe();
    let mut imu_ready_rx = channels.imu.parsed_tx.subscribe();
    let dc_tx = channels.control.dc_motor_tx.clone();

    let mut tick = time::interval(calib.control_period);

    // 가장 최근의 센싱 정보를 보관해 제어 주기마다 활용한다.
    let mut latest_distance: Option<DtoUltraSonicRaw> = None;
    let mut latest_obstacle: Option<Arc<DtoUltraSonicObstacle>> = None;
    let mut latest_signal: Option<DtoTrafficLight> = None;
    let mut latest_directive: Option<Arc<DtoTrafficLightDirective>> = None;
    let mut latest_path: Option<Arc<DtoAdasSmoothedPath>> = None;
    let mut latest_arrival: Option<Arc<DtoLocalizationArrival>> = None;
    let mut last_cmd: Option<(u32, u32)> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();
    let mut stop_request_since: Option<Instant> = None;
    let mut stop_release_since: Option<Instant> = None;
    let mut stop_engaged = false;
    let mut ev_ready_released = false;
    let mut ev_ready_deadline: Option<Instant> = None;
    const EV_READY_HOLD_SECS: u64 = 3;

    loop {
        if !ev_ready_released {
            let mut saw_ready_signal = false;
            loop {
                match imu_ready_rx.try_recv() {
                    Ok(_) => {
                        saw_ready_signal = true;
                        continue;
                    }
                    Err(TryRecvError::Lagged(_)) => {
                        saw_ready_signal = true;
                        break;
                    }
                    Err(TryRecvError::Empty) => {
                        break;
                    }
                    Err(TryRecvError::Closed) => {
                        ev_ready_released = true;
                        println!(
                            "[{}] Longitudinal: IMU channel closed, EV READY 게이트를 해제합니다.",
                            id
                        );
                        break;
                    }
                }
            }
            if saw_ready_signal && ev_ready_deadline.is_none() {
                ev_ready_deadline = Some(Instant::now() + Duration::from_secs(EV_READY_HOLD_SECS));
                println!(
                    "[{}] Longitudinal: EV READY 감지, {}초 대기 후 주행을 시작합니다.",
                    id, EV_READY_HOLD_SECS
                );
            }
            if let Some(deadline) = ev_ready_deadline {
                if Instant::now() >= deadline {
                    ev_ready_released = true;
                    println!(
                        "[{}] Longitudinal: EV READY 게이트 해제, 주행 제어를 시작합니다.",
                        id
                    );
                }
            }
        }

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
                let mut newest = dto;
                while let Ok(newer) = obstacle_rx.try_recv() {
                    newest = newer;
                }
                latest_obstacle = Some(newest);
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
                let mut newest = dto;
                while let Ok(newer) = traffic_directive_rx.try_recv() {
                    newest = newer;
                }
                latest_directive = Some(newest);
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
                let mut newest = dto;
                while let Ok(newer) = path_rx.try_recv() {
                    newest = newer;
                }
                latest_path = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal smoothed_path lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }
        match arrival_rx.try_recv() {
            Ok(dto) => {
                let mut newest = dto;
                while let Ok(newer) = arrival_rx.try_recv() {
                    newest = newer;
                }
                latest_arrival = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal arrival lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        tick.tick().await;

        // 가장 최근 거리 값과 임계치 비교
        let distance_cm = latest_obstacle
            .as_deref()
            .map(|o| o.distance_cm)
            .or_else(|| latest_distance.as_ref().map(|d| d.distance));
        let distance_state = match distance_cm {
            Some(distance) => (
                distance <= calib.stop_distance_cm,
                distance <= calib.slowdown_distance_cm,
            ),
            None => (true, true),
        };
        let obstacle_status = latest_obstacle.as_deref();
        let stop_requested_obstacle = obstacle_status.map(|d| d.stop_requested).unwrap_or(false);
        let stop_requested_signal = latest_directive
            .as_deref()
            .map(|d| d.stop_requested && d.inside_detection_zone)
            .unwrap_or(false);
        let lane_change_requested = obstacle_status
            .map(|d| d.lane_change_requested)
            .unwrap_or(false);
        let accelerate_requested = latest_directive
            .as_deref()
            .map(|d| d.accelerate_requested && d.inside_detection_zone)
            .unwrap_or(false);
        let traffic_color = latest_signal
            .as_ref()
            .map(|signal| signal.traffic_light_color.clone());
        let arrival_state = latest_arrival
            .as_deref()
            .map(|arrival| arrival.arrived)
            .unwrap_or(false);

        // 장애물·신호·거리 조건을 종합해 정지 여부를 결정한다.
        let obstacle_stop = stop_requested_obstacle || distance_state.0;
        let traffic_stop =
            matches!(traffic_color.as_ref(), Some(TrafficLightColor::Red)) || stop_requested_signal;
        let arrival_stop = arrival_state;

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
            .as_deref()
            .map(|path| !path.samples_xy.is_empty())
            .unwrap_or(false);
        let gating_reason = if !ev_ready_released {
            Some("ev_ready")
        } else if !path_ready {
            Some("no_path")
        } else {
            None
        };
        let gating_stop = gating_reason.is_some();

        let curvature_abs = latest_path
            .as_deref()
            .and_then(|path| curvature_from_smoothed_path(path).ok())
            .map(|curv| curv.abs())
            .unwrap_or(0.0);

        let mut gain = 1.0;
        if curvature_abs > calib.curvature_slowdown_threshold {
            gain *= 0.8;
        }

        let base_stop_reason = if arrival_stop {
            Some("arrival")
        } else if obstacle_stop {
            Some("obstacle")
        } else if traffic_stop {
            Some("traffic")
        } else {
            None
        };
        let effective_stop_reason = if let Some(reason) = gating_reason {
            Some(reason)
        } else {
            base_stop_reason
        };

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

        if stop_engaged || gating_stop || effective_stop_reason.is_some() {
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
            let stop_display = if stop_engaged || gating_stop || effective_stop_reason.is_some() {
                effective_stop_reason.unwrap_or("--")
            } else {
                "--"
            };
            let arrival_display = latest_arrival
                .as_deref()
                .map(|arrival| format!("{:.2}", arrival.distance_m))
                .unwrap_or_else(|| "--".to_string());
            println!(
                "[{}] Longitudinal: dist={}cm arrival={} arrival_dist={}m curvature={:.4} gain={:.2} stop={} lane_change={} accel_req={} path_ready={} signal={:?} -> dir={} speed={}",
                id,
                distance_str,
                arrival_state,
                arrival_display,
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

fn compute_lateral_error(
    state: &DtoLocalizationState,
    smoothed_path: &DtoAdasSmoothedPath,
    sample_index: usize,
) -> Option<f64> {
    if smoothed_path.samples_xy.is_empty() {
        return None;
    }

    let idx_sample = sample_index.min(smoothed_path.samples_xy.len().saturating_sub(1));
    let sample = smoothed_path.samples_xy.get(idx_sample)?;

    let reference_pos = state.position_map_xy;
    let heading = if state.yaw_rad.is_finite() {
        state.yaw_rad
    } else {
        state.motion_heading_rad.unwrap_or(0.0)
    };
    let mut tangent = [heading.cos(), heading.sin()];
    let norm = (tangent[0] * tangent[0] + tangent[1] * tangent[1]).sqrt();
    if norm < 1e-6 {
        tangent = [1.0, 0.0];
    } else {
        tangent[0] /= norm;
        tangent[1] /= norm;
    }

    let normal = [-tangent[1], tangent[0]];

    let actual = [sample[0] as f64, sample[1] as f64];
    let diff = [actual[0] - reference_pos[0], actual[1] - reference_pos[1]];
    Some(diff[0] * normal[0] + diff[1] * normal[1])
}
