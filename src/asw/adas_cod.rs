//! ADAS(Advanced Driver Assistance System) 제어 모듈.
//! - `runnable_adas_lateral`: 차선 추정 결과를 이용해 조향 서보를 제어한다.
//! - `runnable_adas_longitudinal`: 장애물과 신호 정보를 사용해 종방향 속도를 결정한다.
use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{AdasLateralCalibration, AdasLongitudinalCalibration};
use crate::rte::rte_dto::{
    AdasLaneChangeState, DtoAdasSmoothedPath, DtoCamLaneAngle, DtoDcMotorCtrl,
    DtoLocalizationArrival, DtoLocalizationState, DtoServoCtrl, DtoTrafficLight,
    DtoTrafficLightDirective, DtoUltraSonicObstacle,
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
    let control_period = Duration::from_millis(100);
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
    let mut pid_output: f64 = 0.0;

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
        let mut lane_angle_allowed = true;

        let mut lateral_error = None;

        if let (Some(state), Some(smoothed_path)) =
            (latest_state.as_deref(), latest_path.as_deref())
        {
            lateral_error = compute_lateral_error(state, smoothed_path, calib.pid_sample_index);
        }

        if let Some(state) = lane_state {
            if !matches!(
                state,
                AdasLaneChangeState::InnerCruise | AdasLaneChangeState::OuterCruise
            ) {
                integral_error = 0.0;
                prev_error = None;
                lateral_error = Some(0.0);
                lane_angle_allowed = false;
            }
        }

        let lane_offset_px = latest_lane_angle
            .as_ref()
            .map(|lane| lane.lateral_offset)
            .unwrap_or(0.0);
        let lane_angle_deg = latest_lane_angle
            .as_ref()
            .map(|lane| lane.angle)
            .unwrap_or(0.0);
        let lane_angle_term = if lane_angle_allowed {
            calib.w_lane_angle * lane_angle_deg
        } else {
            0.0
        };
        let mut control_error =
            lateral_error.map(|err| calib.w_lateral_error * err + lane_angle_term);
        if control_error.is_none() && lane_angle_term.abs() > f64::EPSILON {
            control_error = Some(lane_angle_term);
        }

        if let Some(error) = control_error {
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

        let base_cmd = calib.servo_neutral_deg as f64 + pid_output;

        //let base_cmd = calib.servo_neutral_deg as f64
        //    - 450.0 * control_error.unwrap_or(0.0);
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

        //println!("[ADAS-COD] Lateral error : {:?}", lateral_error);
        //println!("[ADAS-COD] total cmd : {}", limited_deg);

        // 명령 송신: DTO를 Arc로 감싸 브로드캐스트한다.
        let dto = DtoServoCtrl::new(calib.servo_channel_index, limited_deg);
        let _ = servo_tx.send(std::sync::Arc::new(dto));
        last_cmd_deg = limited_deg;

        // 1초마다 현재 제어 상태를 요약해 로깅한다.
        if last_log.elapsed() > std::time::Duration::from_secs(1) {
            let error_display = control_error
                .map(|err| format!("{:.3}", err))
                .unwrap_or_else(|| "--".to_string());
            let lateral_display = lateral_error
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
                "[{}] Lateral: yaw={} state={} -> servo={}deg error={} lat_err={} angle={:.2}deg offset={}",
                id,
                yaw_display,
                state_display,
                last_cmd_deg,
                error_display,
                lateral_display,
                lane_angle_deg,
                lane_offset_px
            );
            last_log = Instant::now();
        }
    }
}

/// ADAS Longitudinal 제어 러너블.
/// - 장애물/신호등/목적지 도달 조건을 상태로 평가하고, 정지 상태가 아니면 목표 속도를 추종한다.
/// - Localization 기반 속도 추정 + PID 제어를 사용해 DC 모터 듀티를 결정한다.
pub async fn runnable_adas_longitudinal(id: &'static str, channels: RteChannels) {
    let calib = AdasLongitudinalCalibration::default();

    let mut localization_rx = channels.localization.state_tx.subscribe();
    let mut obstacle_rx = channels.ultrasonic.obstacle_tx.subscribe();
    let mut traffic_rx = channels.camera.traffic_light_tx.subscribe();
    let mut traffic_directive_rx = channels.camera.traffic_light_directive_tx.subscribe();
    let mut arrival_rx = channels.localization.arrival_tx.subscribe();
    let mut imu_ready_rx = channels.imu.parsed_tx.subscribe();
    // 서보 각도(조향각)도 함께 구독해 속도 목표를 조절한다.
    let mut servo_rx = channels.control.servo_tx.subscribe();
    let dc_tx = channels.control.dc_motor_tx.clone();

    let mut tick = time::interval(calib.control_period);

    let mut latest_state: Option<Arc<DtoLocalizationState>> = None;
    let mut latest_obstacle: Option<Arc<DtoUltraSonicObstacle>> = None;
    let mut latest_signal: Option<DtoTrafficLight> = None;
    let mut latest_directive: Option<Arc<DtoTrafficLightDirective>> = None;
    let mut latest_arrival: Option<Arc<DtoLocalizationArrival>> = None;

    let mut last_cmd: Option<(u32, u32)> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();
    let mut ev_ready_released = false;
    let mut ev_ready_deadline: Option<Instant> = None;
    let mut measured_speed_mps: f64 = 0.0;
    let mut prev_speed_sample: Option<([f64; 2], u64)> = None;
    let mut speed_pid_integral: f64 = 0.0;
    let mut speed_pid_prev_error: Option<f64> = None;
    let mut latest_servo_angle_deg: Option<u32> = None;
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

        match localization_rx.try_recv() {
            Ok(dto) => {
                let mut newest = dto;
                while let Ok(newer) = localization_rx.try_recv() {
                    newest = newer;
                }
                latest_state = Some(newest);
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal localization lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        match obstacle_rx.try_recv() {
            Ok(dto) => {
                latest_obstacle = Some(dto.clone());
                while let Ok(newer) = obstacle_rx.try_recv() {
                    latest_obstacle = Some(newer.clone());
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

        match traffic_directive_rx.try_recv() {
            Ok(dto) => {
                latest_directive = Some(dto.clone());
                while let Ok(newer) = traffic_directive_rx.try_recv() {
                    latest_directive = Some(newer);
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

        match arrival_rx.try_recv() {
            Ok(dto) => {
                latest_arrival = Some(dto);
                while let Ok(newer) = arrival_rx.try_recv() {
                    latest_arrival = Some(newer);
                }
            }
            Err(TryRecvError::Lagged(n)) => {
                eprintln!("[{}] ADAS longitudinal arrival lagged by {}", id, n);
            }
            Err(TryRecvError::Closed) | Err(TryRecvError::Empty) => {}
        }

        // 최신 서보 각도(조향)도 드레인해 캐시한다.
        loop {
            match servo_rx.try_recv() {
                Ok(dto) => {
                    latest_servo_angle_deg = Some(dto.as_ref().angle);
                    continue;
                }
                Err(TryRecvError::Lagged(_)) => {
                    // 가장 최신 값만 유지하면 되므로 무시하고 계속
                    continue;
                }
                Err(TryRecvError::Empty) | Err(TryRecvError::Closed) => break,
            }
        }

        tick.tick().await;

        if let Some(state) = latest_state.as_deref() {
            if let Some((prev_pos, prev_ts)) = prev_speed_sample {
                if state.timestamp_ns > prev_ts {
                    let dt = (state.timestamp_ns - prev_ts) as f64 * 1e-9;
                    if dt > 0.0 {
                        let dx = state.position_map_xy[0] - prev_pos[0];
                        let dy = state.position_map_xy[1] - prev_pos[1];
                        let dist = (dx * dx + dy * dy).sqrt();
                        let speed = dist / dt;
                        if speed.is_finite() {
                            measured_speed_mps = speed;
                        }
                    }
                }
            }
            prev_speed_sample = Some((state.position_map_xy, state.timestamp_ns));
        }

        let obstacle_distance_cm = latest_obstacle
            .as_deref()
            .map(|o| o.distance_cm)
            .unwrap_or(f32::MAX);
        let obstacle_stop = obstacle_distance_cm <= 20.0;

        let traffic_color = latest_signal
            .as_ref()
            .map(|signal| signal.traffic_light_color.clone());
        let in_detection_zone = latest_directive
            .as_deref()
            .map(|d| d.inside_detection_zone)
            .unwrap_or(false);
        let traffic_stop =
            in_detection_zone && matches!(traffic_color.as_ref(), Some(TrafficLightColor::Red));

        let arrival_stop = latest_arrival
            .as_deref()
            .map(|arrival| arrival.arrived)
            .unwrap_or(false);

        let should_stop = obstacle_stop || traffic_stop || arrival_stop;

        let mut target_speed_mps = if !ev_ready_released || should_stop {
            0.0
        } else {
            calib.speed_target_mps
        };

        // 조향각이 75~105도 사이면 목표 속도를 0.15로 제한한다.
        if target_speed_mps > 0.0 && calib.steer_slow_speed_mps > 0.0 {
            if let Some(servo_deg) = latest_servo_angle_deg {
                let min_deg = calib
                    .steer_slow_min_deg
                    .min(calib.steer_slow_max_deg);
                let max_deg = calib
                    .steer_slow_min_deg
                    .max(calib.steer_slow_max_deg);
                if (min_deg..=max_deg).contains(&servo_deg) {
                    target_speed_mps = target_speed_mps.min(calib.steer_slow_speed_mps);
                }
            }
        }

        let feedforward_percent = if target_speed_mps <= 0.0 || calib.speed_target_mps <= 0.0 {
            0.0
        } else {
            target_speed_mps
        };

        let mut commanded_percent = if target_speed_mps <= 0.0 {
            speed_pid_integral = 0.0;
            speed_pid_prev_error = None;
            0.0
        } else {
            let error = target_speed_mps - measured_speed_mps;
            let control_dt = calib.control_period.as_secs_f64();
            speed_pid_integral += error * control_dt;
            if calib.speed_pid_integral_limit > 0.0 {
                let limit = calib.speed_pid_integral_limit;
                speed_pid_integral = speed_pid_integral.clamp(-limit, limit);
            }
            let derivative = if let Some(prev) = speed_pid_prev_error {
                (error - prev) / control_dt
            } else {
                0.0
            };
            speed_pid_prev_error = Some(error);
            feedforward_percent
                + calib.speed_pid_kp * error
                + calib.speed_pid_ki * speed_pid_integral
                + calib.speed_pid_kd * derivative
        };

        commanded_percent = commanded_percent.clamp(0.0, 50.0);
        let commanded_percent = commanded_percent.round() as u32;

        let desired_command = if commanded_percent == 0 {
            (0, 0)
        } else {
            (1, commanded_percent)
        };

        let previous_speed = last_cmd.map(|(_, spd)| spd).unwrap_or(0);
        let limited_speed = apply_speed_rate_limit(previous_speed, desired_command.1, &calib);
        let limited_direction = if limited_speed == 0 { 0 } else { 1 };
        let command = (limited_direction, limited_speed);

        if last_cmd.map(|prev| prev != command).unwrap_or(true) {
            let dto = DtoDcMotorCtrl::new(command.0, command.1, alive_cnt);
            let _ = dc_tx.send(Arc::new(dto));
            alive_cnt = alive_cnt.wrapping_add(1);
            last_cmd = Some(command);
        }

        if last_log.elapsed() >= calib.log_interval {
            println!(
                "[{}] Longitudinal: stop={} obstacle={:.1}cm traffic={:?} arrival={} speed={:.2}m/s target={:.2}m/s cmd=({}, {})",
                id,
                should_stop,
                obstacle_distance_cm,
                traffic_color,
                arrival_stop,
                measured_speed_mps,
                target_speed_mps,
                command.0,
                command.1
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
