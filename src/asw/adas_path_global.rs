//! ADAS 전역 경로 탐색 러너블(상태머신 기반 버전).
//! 장애물·차선 변경 요청과 주기 타이머를 상태로 관리해 재계획 시점을 단순화한다.

use crate::asw::lib::adas_path_lib::{NodeKey, PathGraph, PathPlanningMode, publish_global_path};
use crate::calibration::{AdasPathGlobalCalibration, LOCALIZATION_ACTIVE_SCENARIO};
use crate::rte::rte_dto::{
    AdasLaneChangeState, DtoAdasGlobalPath, DtoAdasSmoothedPath, DtoLocalizationState,
    DtoUltraSonicObstacle,
};
use crate::rte::rte_main::RteChannels;
use std::collections::HashSet;
use std::f64::consts::PI;
use std::time::{Duration, Instant};
use tokio::sync::broadcast::error::RecvError;
use tokio::time::{self, MissedTickBehavior};

const LANE_CHANGE_HEADING_TOL_RAD: f64 = 15.0_f64 * PI / 180.0;
const LANE_CHANGE_LATERAL_TOL_M: f64 = 0.03;
const LANE_CHANGE_SETTLE_DURATION: Duration = Duration::from_secs(2);
const LANE_CHANGE_REQUEST_DISTANCE_CM: f32 = 55.0;

pub async fn runnable_adas_path_global(id: &'static str, channels: RteChannels) {
    let calib = AdasPathGlobalCalibration::default();
    let scenario = LOCALIZATION_ACTIVE_SCENARIO;

    let graph = match PathGraph::load(scenario.map, &calib) {
        Ok(graph) => graph,
        Err(err) => {
            eprintln!("[{}] 전역 경로용 지도 로딩 실패: {}", id, err);
            return;
        }
    };

    let goal_key = NodeKey {
        lane: scenario.destination.lane,
        index: scenario.destination.waypoint_index,
    };
    if graph.waypoint(goal_key).is_none() {
        eprintln!(
            "[{}] 목적지 waypoint(lane={:?}, index={})가 지도 범위 밖입니다.",
            id, goal_key.lane, goal_key.index
        );
        return;
    }

    println!(
        "[{}] 전역 경로 러너블 시작: map={:?}, destination=({:?}, index={})",
        id,
        graph.map_id(),
        goal_key.lane,
        goal_key.index
    );

    let mut localization_rx = channels.localization.state_tx.subscribe();
    let mut obstacle_rx = channels.ultrasonic.obstacle_tx.subscribe();
    let mut smoothed_rx = channels.path.smoothed_tx.subscribe();
    let path_tx = channels.path.global_tx.clone();

    let mut latest_state: Option<DtoLocalizationState> = None;
    let blocked_cache: HashSet<NodeKey> = HashSet::new();

    let mut lateral_sm = LateralStateMachine::new();
    let mut lane_change_request_active = false;
    let mut lane_change_sensor_active = false;

    let mut tick = time::interval(calib.replanning_period);
    tick.set_missed_tick_behavior(MissedTickBehavior::Delay);

    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();

    loop {
        tokio::select! {
            msg = localization_rx.recv() => {
                match msg {
                    Ok(state_arc) => {
                        let mut newest = state_arc;
                        while let Ok(newer) = localization_rx.try_recv() {
                            newest = newer;
                        }
                        latest_state = Some(newest.as_ref().clone());
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] Localization 업데이트 {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] Localization 채널 종료, 전역 경로 러너블 종료", id);
                        break;
                    }
                }
            }
            msg = obstacle_rx.recv() => {
                match msg {
                    Ok(obstacle_arc) => {
                        let mut newest = obstacle_arc;
                        while let Ok(newer) = obstacle_rx.try_recv() {
                            newest = newer;
                        }
                        let obstacle: DtoUltraSonicObstacle = newest.as_ref().clone();
                        lane_change_sensor_active =
                            obstacle.distance_cm <= LANE_CHANGE_REQUEST_DISTANCE_CM;

                        if lane_change_sensor_active {
                            if !lane_change_request_active {
                                println!(
                                    "[{}] 장애물 감지: {:.1}cm (lane-change 요청 시작)",
                                    id, obstacle.distance_cm
                                );
                            }
                            lane_change_request_active = true;
                            if matches!(lateral_sm.state(), LateralDrivingState::LaneKeeping) {
                                let _ = try_publish_path(
                                    id,
                                    &graph,
                                    &calib,
                                    &path_tx,
                                    latest_state.as_ref(),
                                    goal_key,
                                    &blocked_cache,
                                    &mut alive_cnt,
                                    &mut last_log,
                                    PathPlanningMode::ForceLaneChange,
                                );
                            }
                        } else if matches!(lateral_sm.state(), LateralDrivingState::LaneKeeping) {
                            if lane_change_request_active {
                                println!(
                                    "[{}] 장애물 해제: lane-change 요청을 종료합니다.",
                                    id
                                );
                            }
                            lane_change_request_active = false;
                        }
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] 장애물 업데이트 {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] 장애물 채널 종료, 전역 경로 러너블 종료", id);
                        break;
                    }
                }
            }
            msg = smoothed_rx.recv() => {
                match msg {
                    Ok(smoothed_arc) => {
                        let now = Instant::now();
                        let (prev_state, new_state) = lateral_sm.update(
                            smoothed_arc.as_ref(),
                            latest_state.as_ref(),
                            now,
                        );
                        if prev_state != new_state {
                            println!(
                                "[{}] 횡방향 상태 전이: {:?} -> {:?}",
                                id, prev_state, new_state
                            );
                            if matches!(new_state, LateralDrivingState::LaneKeeping)
                                && !lane_change_sensor_active
                                && lane_change_request_active
                            {
                                println!(
                                    "[{}] 차선 변경 완료 감지: lane-change 요청을 해제합니다.",
                                    id
                                );
                                lane_change_request_active = false;
                            }
                        }
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] 스무딩 경로 업데이트 {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] 스무딩 경로 채널 종료, 전역 경로 러너블 종료", id);
                        break;
                    }
                }
            }
            _ = tick.tick() => {
                if matches!(lateral_sm.state(), LateralDrivingState::LaneKeeping) {
                    let mode = if lane_change_request_active {
                        PathPlanningMode::ForceLaneChange
                    } else {
                        PathPlanningMode::Normal
                    };
                    let _ = try_publish_path(
                        id,
                        &graph,
                        &calib,
                        &path_tx,
                        latest_state.as_ref(),
                        goal_key,
                        &blocked_cache,
                        &mut alive_cnt,
                        &mut last_log,
                        mode,
                    );
                }
            }
        }
    }
}

fn try_publish_path(
    id: &str,
    graph: &PathGraph,
    calib: &AdasPathGlobalCalibration,
    path_tx: &tokio::sync::broadcast::Sender<std::sync::Arc<DtoAdasGlobalPath>>,
    state: Option<&DtoLocalizationState>,
    goal_key: NodeKey,
    blocked: &HashSet<NodeKey>,
    alive_cnt: &mut u32,
    last_log: &mut Instant,
    mode: PathPlanningMode,
) {
    let Some(state) = state else {
        return;
    };
    if let Err(err) = publish_global_path(
        id, graph, calib, path_tx, state, goal_key, blocked, alive_cnt, last_log, mode,
    ) {
        eprintln!("[{}] 전역 경로 생성 실패({:?}): {}", id, mode, err);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LateralDrivingState {
    LaneKeeping,
    LaneChanging,
    LaneChangeSettled,
}

struct LateralStateMachine {
    state: LateralDrivingState,
    settle_since: Option<Instant>,
}

impl LateralStateMachine {
    fn new() -> Self {
        Self {
            state: LateralDrivingState::LaneKeeping,
            settle_since: None,
        }
    }

    fn state(&self) -> LateralDrivingState {
        self.state
    }

    fn update(
        &mut self,
        path: &DtoAdasSmoothedPath,
        localization: Option<&DtoLocalizationState>,
        now: Instant,
    ) -> (LateralDrivingState, LateralDrivingState) {
        let prev = self.state;
        match self.state {
            LateralDrivingState::LaneKeeping => {
                if path_requires_lane_change(path) {
                    self.state = LateralDrivingState::LaneChanging;
                    self.settle_since = None;
                }
            }
            LateralDrivingState::LaneChanging => {
                if let Some((lateral_err, heading_err)) =
                    localization.and_then(|state| lane_alignment_error(state, path))
                {
                    let within_heading = heading_err <= LANE_CHANGE_HEADING_TOL_RAD;
                    let within_lateral = lateral_err.abs() <= LANE_CHANGE_LATERAL_TOL_M;
                    if within_heading && within_lateral {
                        if let Some(started) = self.settle_since {
                            if now.duration_since(started) >= LANE_CHANGE_SETTLE_DURATION {
                                self.state = LateralDrivingState::LaneChangeSettled;
                                self.settle_since = None;
                            }
                        } else {
                            self.settle_since = Some(now);
                        }
                    } else {
                        self.settle_since = None;
                    }
                } else {
                    self.settle_since = None;
                }
            }
            LateralDrivingState::LaneChangeSettled => {
                self.state = LateralDrivingState::LaneKeeping;
                self.settle_since = None;
            }
        }
        (prev, self.state)
    }
}

fn path_requires_lane_change(path: &DtoAdasSmoothedPath) -> bool {
    matches!(
        path.lane_change_state,
        AdasLaneChangeState::InnerToOuter | AdasLaneChangeState::OuterToInner
    )
}

fn lane_alignment_error(
    state: &DtoLocalizationState,
    path: &DtoAdasSmoothedPath,
) -> Option<(f64, f64)> {
    let sample = path.samples_xy.first()?;
    let state_heading = if state.yaw_rad.is_finite() {
        state.yaw_rad
    } else {
        state.motion_heading_rad?
    };

    let path_vec = path
        .samples_xy
        .get(1)
        .map(|next| [(next[0] - sample[0]) as f64, (next[1] - sample[1]) as f64]);

    let tangent = path_vec
        .and_then(normalize_vec)
        .unwrap_or([state_heading.cos(), state_heading.sin()]);
    let path_heading = path_vec
        .map(|vec| vec[1].atan2(vec[0]))
        .unwrap_or(state_heading);

    let normal = [-tangent[1], tangent[0]];
    let diff = [
        sample[0] as f64 - state.position_map_xy[0],
        sample[1] as f64 - state.position_map_xy[1],
    ];
    let lateral = diff[0] * normal[0] + diff[1] * normal[1];
    let heading_err = angle_difference(path_heading, state_heading).abs();
    Some((lateral, heading_err))
}

fn normalize_vec(mut vec: [f64; 2]) -> Option<[f64; 2]> {
    let norm = (vec[0] * vec[0] + vec[1] * vec[1]).sqrt();
    if norm <= 1e-6 {
        None
    } else {
        vec[0] /= norm;
        vec[1] /= norm;
        Some(vec)
    }
}

fn angle_difference(a: f64, b: f64) -> f64 {
    let mut diff = a - b;
    while diff > PI {
        diff -= 2.0 * PI;
    }
    while diff < -PI {
        diff += 2.0 * PI;
    }
    diff
}
