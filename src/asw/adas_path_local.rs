//! ADAS 단기 경로 생성 러너블.
//! 전역 경로 결과를 구독해 차량 주변 구간(기본 10개 waypoint)을 잘라서 RTE에 배포한다.

use crate::asw::lib::adas_path_lib::{now_timestamp_ns, smooth_local_path, try_publish_local_path};
use crate::calibration::{
    AdasPathLocalCalibration, LOCALIZATION_ACTIVE_SCENARIO, LocalizationLane,
};
use crate::rte::rte_dto::{
    AdasLaneChangeState, DtoAdasGlobalPath, DtoAdasLocalPath, DtoAdasSmoothedPath,
    DtoLocalizationState,
};
use crate::rte::rte_main::RteChannels;
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_adas_path_local(id: &'static str, channels: RteChannels) {
    let calib = AdasPathLocalCalibration::default();
    let scenario = LOCALIZATION_ACTIVE_SCENARIO;

    let mut global_rx = channels.path.global_tx.subscribe();
    let mut localization_rx = channels.localization.state_tx.subscribe();
    let local_tx = channels.path.local_tx.clone();

    let mut latest_global: Option<Arc<DtoAdasGlobalPath>> = None;
    let mut latest_state: Option<Arc<DtoLocalizationState>> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();

    println!(
        "[{}] 로컬 경로 러너블 시작: map={:?}, window={}",
        id, scenario.map, calib.waypoint_window
    );

    loop {
        tokio::select! {
            msg = global_rx.recv() => {
                match msg {
                    Ok(path_arc) => {
                        latest_global = Some(path_arc);
                        try_publish_local_path(
                            id,
                            &calib,
                            &local_tx,
                            latest_global.as_ref(),
                            latest_state.as_deref(),
                            &mut alive_cnt,
                            &mut last_log,
                        );
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] 전역 경로 메시지 {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] 전역 경로 채널 종료, 로컬 경로 러너블 종료", id);
                        break;
                    }
                }
            }
            msg = localization_rx.recv() => {
                match msg {
                    Ok(state_arc) => {
                        let mut newest = state_arc;
                        while let Ok(newer) = localization_rx.try_recv() {
                            newest = newer;
                        }
                        latest_state = Some(newest);
                        try_publish_local_path(
                            id,
                            &calib,
                            &local_tx,
                            latest_global.as_ref(),
                            latest_state.as_deref(),
                            &mut alive_cnt,
                            &mut last_log,
                        );
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] Localization {}개 누락 (로컬 경로)", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] Localization 채널 종료, 로컬 경로 러너블 종료", id);
                        break;
                    }
                }
            }
        }
    }
}

/// 로컬 경로를 기반으로 5차 다항식 스무딩 궤적을 생성해 전파한다.
pub async fn runnable_adas_path_smoothing(id: &'static str, channels: RteChannels) {
    let calib = AdasPathLocalCalibration::default();

    let mut local_rx = channels.path.local_tx.subscribe();
    let mut localization_rx = channels.localization.state_tx.subscribe();
    let smooth_tx = channels.path.smoothed_tx.clone();

    let mut latest_local: Option<Arc<DtoAdasLocalPath>> = None;
    let mut latest_state: Option<Arc<DtoLocalizationState>> = None;
    let mut alive_cnt: u32 = 0;
    let mut last_log = Instant::now();

    println!(
        "[{}] 로컬 궤적 스무딩 러너블 시작: samples={}",
        id, calib.smoothing_sample_count
    );

    loop {
        tokio::select! {
            msg = local_rx.recv() => {
                match msg {
                    Ok(path_arc) => {
                        latest_local = Some(path_arc);
                        try_publish_smoothed_path(
                            id,
                            &calib,
                            &smooth_tx,
                            latest_local.as_ref(),
                            latest_state.as_deref(),
                            &mut alive_cnt,
                            &mut last_log,
                        );
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] 스무딩: 로컬 경로 {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] 스무딩: 로컬 경로 채널 종료", id);
                        break;
                    }
                }
            }
            msg = localization_rx.recv() => {
                match msg {
                    Ok(state_arc) => {
                        let mut newest = state_arc;
                        while let Ok(newer) = localization_rx.try_recv() {
                            newest = newer;
                        }
                        latest_state = Some(newest);
                        try_publish_smoothed_path(
                            id,
                            &calib,
                            &smooth_tx,
                            latest_local.as_ref(),
                            latest_state.as_deref(),
                            &mut alive_cnt,
                            &mut last_log,
                        );
                    }
                    Err(RecvError::Lagged(skipped)) => {
                        eprintln!("[{}] 스무딩: Localization {}개 누락", id, skipped);
                    }
                    Err(RecvError::Closed) => {
                        println!("[{}] 스무딩: Localization 채널 종료", id);
                        break;
                    }
                }
            }
        }
    }
}

fn try_publish_smoothed_path(
    id: &str,
    calib: &AdasPathLocalCalibration,
    smooth_tx: &tokio::sync::broadcast::Sender<Arc<DtoAdasSmoothedPath>>,
    local_path: Option<&Arc<DtoAdasLocalPath>>,
    state: Option<&DtoLocalizationState>,
    alive_cnt: &mut u32,
    last_log: &mut Instant,
) {
    let (local_path, state) = match (local_path, state) {
        (Some(path), Some(state)) => (path, state),
        _ => return,
    };

    let lane_state = determine_lane_change_state(local_path);

    let skip = calib.smoothing_skip_head.min(local_path.waypoints.len());
    let smoothing_input = &local_path.waypoints[skip..];
    let smoothing_source = if smoothing_input.len() >= 6 {
        smoothing_input
    } else {
        &local_path.waypoints
    };

    let samples = match smooth_local_path(smoothing_source, state, calib.smoothing_sample_count) {
        Ok(samples) => samples,
        Err(err) => {
            eprintln!("[{}] 스무딩 실패: {} (원본 경로로 대체)", id, err);
            local_path
                .waypoints
                .iter()
                .map(|wp| wp.position_xy)
                .collect::<Vec<_>>()
        }
    };

    let sample_len = samples.len();
    let dto = Arc::new(DtoAdasSmoothedPath::new(
        local_path.map_id,
        local_path.origin_alive_cnt,
        samples,
        *alive_cnt,
        now_timestamp_ns(),
        lane_state,
    ));
    if smooth_tx.send(dto).is_ok() {
        *alive_cnt = alive_cnt.wrapping_add(1);
        if last_log.elapsed().as_secs_f32() >= 1.0 {
            println!(
                "[{}] 스무딩 궤적 업데이트: plan_alive={} samples={} state={:?}",
                id, local_path.origin_alive_cnt, sample_len, lane_state
            );
            *last_log = Instant::now();
        }
    }
}

fn determine_lane_change_state(local_path: &DtoAdasLocalPath) -> AdasLaneChangeState {
    let mut has_inner = false;
    let mut has_outer = false;
    let first_lane = local_path.waypoints.first().map(|wp| wp.lane);

    for wp in &local_path.waypoints {
        match wp.lane {
            LocalizationLane::Inner => has_inner = true,
            LocalizationLane::Outer => has_outer = true,
        }
    }

    match (has_inner, has_outer, first_lane) {
        (true, true, Some(LocalizationLane::Inner)) => AdasLaneChangeState::InnerToOuter,
        (true, true, Some(LocalizationLane::Outer)) => AdasLaneChangeState::OuterToInner,
        (true, false, _) => AdasLaneChangeState::InnerCruise,
        (false, true, _) => AdasLaneChangeState::OuterCruise,
        _ => match first_lane.unwrap_or(LocalizationLane::Inner) {
            LocalizationLane::Inner => AdasLaneChangeState::InnerCruise,
            LocalizationLane::Outer => AdasLaneChangeState::OuterCruise,
        },
    }
}
