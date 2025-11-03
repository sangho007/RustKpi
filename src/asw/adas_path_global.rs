//! ADAS 전역 경로 탐색 러너블.
//! Localization 결과를 기반으로 출발 waypoint를 결정하고
//! Python A* 구현을 포팅한 라이브러리(`adas_path_lib`)를 호출해 목적지까지의 경로를 생성한다.

use crate::asw::lib::adas_path_lib::{
    NodeKey, PathGraph, PathPlanningMode, PlannedPath, publish_global_path,
};
use crate::calibration::{AdasPathGlobalCalibration, LOCALIZATION_ACTIVE_SCENARIO};
use crate::rte::rte_dto::{DtoLocalizationState, DtoUltraSonicObstacle};
use crate::rte::rte_main::RteChannels;
use std::collections::{HashMap, HashSet};
use std::time::Instant;
use tokio::sync::broadcast::error::RecvError;
use tokio::time::{self, MissedTickBehavior};

pub async fn runnable_adas_path_global(id: &'static str, channels: RteChannels) {
    let calib = AdasPathGlobalCalibration::default();
    let scenario = LOCALIZATION_ACTIVE_SCENARIO;

    let heading_cos_threshold =
        (calib.obstacle_block_heading_tolerance_deg as f64).to_radians().cos();

    let graph = match PathGraph::load(scenario.map, &calib) {
        Ok(graph) => graph,
        Err(err) => {
            eprintln!("[{}] 전역 경로용 지도 로딩 실패: {}", id, err);
            return;
        }
    };

    // 목적지 waypoint 유효성 확인.
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
    let path_tx = channels.path.global_tx.clone();

    let mut latest_state: Option<DtoLocalizationState> = None;
    let mut latest_plan: Option<PlannedPath> = None;
    let mut blocked_nodes: HashMap<NodeKey, Instant> = HashMap::new();
    let mut blocked_cache: HashSet<NodeKey> = HashSet::new();
    let mut lane_change_requested = false;
    let mut lane_change_cooldown_until: Option<Instant> = None;

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
                        let now = Instant::now();
                        refresh_blocked_nodes(&mut blocked_nodes, &mut blocked_cache, now);

                        lane_change_requested = obstacle.lane_change_requested;
                        if !obstacle.lane_change_requested && !obstacle.stop_requested {
                            blocked_nodes.clear();
                            blocked_cache.clear();
                            lane_change_cooldown_until = None;
                            continue;
                        }

                        if obstacle.lane_change_requested {
                            if let (Some(plan), Some(state)) =
                                (latest_plan.as_ref(), latest_state.as_ref())
                            {
                                let threshold =
                                    (obstacle.distance_cm as f64 / 100.0f64).max(0.0)
                                        + calib.obstacle_block_margin_m as f64;
                                for node in compute_blocked_nodes(
                                    plan,
                                    state,
                                    threshold,
                                    heading_cos_threshold,
                                ) {
                                    blocked_nodes
                                        .insert(node, now + calib.obstacle_block_timeout);
                                }
                                refresh_blocked_nodes(&mut blocked_nodes, &mut blocked_cache, now);
                            }

                            if let Some(state) = latest_state.as_ref() {
                                if lane_change_cooldown_until.map_or(true, |deadline| now >= deadline)
                                {
                                    match publish_global_path(
                                        id,
                                        &graph,
                                        &calib,
                                        &path_tx,
                                        state,
                                        goal_key,
                                        &blocked_cache,
                                        &mut alive_cnt,
                                        &mut last_log,
                                        PathPlanningMode::ForceLaneChange,
                                    ) {
                                        Ok(plan) => {
                                            latest_plan = Some(plan);
                                            lane_change_cooldown_until = None;
                                        }
                                        Err(err) => {
                                            eprintln!(
                                                "[{}] 전역 경로 생성 실패(ForceLaneChange): {}",
                                                id, err
                                            );
                                            lane_change_cooldown_until =
                                                Some(now + calib.lane_change_retry_cooldown);
                                        }
                                    }
                                }
                            }
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
            _ = tick.tick() => {
                if let Some(state) = latest_state.as_ref() {
                    let now = Instant::now();
                    refresh_blocked_nodes(&mut blocked_nodes, &mut blocked_cache, now);

                    let force_allowed = lane_change_requested
                        && lane_change_cooldown_until.map_or(true, |deadline| now >= deadline);
                    let mode = if force_allowed {
                        PathPlanningMode::ForceLaneChange
                    } else {
                        PathPlanningMode::Normal
                    };

                    match publish_global_path(
                        id,
                        &graph,
                        &calib,
                        &path_tx,
                        state,
                        goal_key,
                        &blocked_cache,
                        &mut alive_cnt,
                        &mut last_log,
                        mode,
                    ) {
                        Ok(plan) => {
                            latest_plan = Some(plan);
                            if matches!(mode, PathPlanningMode::ForceLaneChange) {
                                lane_change_cooldown_until = None;
                            }
                        }
                        Err(err) => {
                            eprintln!("[{}] 전역 경로 생성 실패({:?}): {}", id, mode, err);
                            if matches!(mode, PathPlanningMode::ForceLaneChange) {
                                lane_change_cooldown_until =
                                    Some(now + calib.lane_change_retry_cooldown);
                            }
                        }
                    }
                }
            }
        }
    }
}

fn refresh_blocked_nodes(
    blocked: &mut HashMap<NodeKey, Instant>,
    cache: &mut HashSet<NodeKey>,
    now: Instant,
) {
    blocked.retain(|_, &mut expires| expires > now);
    cache.clear();
    cache.extend(blocked.keys().copied());
}

fn compute_blocked_nodes(
    plan: &PlannedPath,
    state: &DtoLocalizationState,
    threshold_m: f64,
    heading_cos_threshold: f64,
) -> Vec<NodeKey> {
    if plan.waypoints.is_empty() {
        return Vec::new();
    }

    let position = state.position_map_xy;
    let mut nearest_idx = 0usize;
    let mut best = f64::INFINITY;
    for (idx, wp) in plan.waypoints.iter().enumerate() {
        let dist = distance_2d(position, [wp.x, wp.y]);
        if dist < best {
            best = dist;
            nearest_idx = idx;
        }
    }

    let heading_unit = state
        .motion_heading_rad
        .map(|ang| [ang.cos(), ang.sin()])
        .or_else(|| heading_from_plan(plan, nearest_idx))
        .and_then(normalize_vec);

    let mut blocked = Vec::new();
    let mut accumulated = 0.0;
    let mut prev = [plan.waypoints[nearest_idx].x, plan.waypoints[nearest_idx].y];

    for wp in plan.waypoints.iter().skip(nearest_idx) {
        if let Some(heading) = heading_unit {
            let vec = [wp.x - position[0], wp.y - position[1]];
            if let Some(dir) = normalize_vec(vec) {
                let dot = dir[0] * heading[0] + dir[1] * heading[1];
                if dot < heading_cos_threshold {
                    continue;
                }
            }
        }

        if blocked.is_empty() {
            blocked.push(NodeKey {
                lane: wp.lane,
                index: wp.index,
            });
            continue;
        }
        let current = [wp.x, wp.y];
        accumulated += distance_2d(prev, current);
        if accumulated <= threshold_m {
            blocked.push(NodeKey {
                lane: wp.lane,
                index: wp.index,
            });
            prev = current;
        } else {
            break;
        }
    }

    blocked
}

fn distance_2d(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
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

fn heading_from_plan(plan: &PlannedPath, idx: usize) -> Option<[f64; 2]> {
    let current = plan.waypoints.get(idx)?;
    if let Some(next) = plan.waypoints.get(idx + 1) {
        Some([next.x - current.x, next.y - current.y])
    } else if let Some(prev_idx) = idx.checked_sub(1) {
        let prev = plan.waypoints.get(prev_idx)?;
        Some([current.x - prev.x, current.y - prev.y])
    } else {
        None
    }
}
