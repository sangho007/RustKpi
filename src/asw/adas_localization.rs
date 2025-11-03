//! ADAS Localization 러너블.
//! 맵 정보를 기반으로 IMU 변위를 통합해 현재 위치와 yaw를 계산한다.
//! 주기적으로 IMU 데이터를 받아 라이브러리 함수에 전달하고,
//! 계산된 `DtoLocalizationState`를 RTE 채널로 브로드캐스트한다.

use crate::asw::lib::adas_localization_lib::{LocalizationRuntime, MapData, process_imu_sample};
use crate::calibration::adas_localization::{
    LOCALIZATION_ACTIVE_SCENARIO, LOCALIZATION_ARRIVAL_THRESHOLD_M,
};
use crate::rte::rte_dto::{DtoLocalizationArrival, DtoLocalizationState};
use crate::rte::rte_main::RteChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

/// ADAS Localization 메인 러너블.
pub async fn runnable_adas_localization(id: &'static str, channels: RteChannels) {
    // 실험·테스트 시나리오에서 선택한 지도/출발지 정보를 불러온다.
    let scenario = LOCALIZATION_ACTIVE_SCENARIO;
    let map = match MapData::load(scenario.map) {
        Ok(map) => map,
        Err(err) => {
            eprintln!(
                "[{}] Localization map({:?}) 로딩 실패: {}",
                id, scenario.map, err
            );
            return;
        }
    };

    let start_coord = match map.waypoint(scenario.start.lane, scenario.start.waypoint_index) {
        Some(coord) => coord,
        None => {
            // 캘리브레이션에서 지정한 waypoint가 잘못된 경우 즉시 종료한다.
            eprintln!(
                "[{}] 시작 지점 waypoint_index={}가 맵 데이터 범위 밖입니다.",
                id, scenario.start.waypoint_index
            );
            return;
        }
    };

    println!(
        "[{}] Localization 시작: map={:?}, start_lane={:?}, start_xy=({:.3}, {:.3})",
        id, scenario.map, scenario.start.lane, start_coord.x, start_coord.y
    );

    let mut imu_rx = channels.imu.parsed_tx.subscribe();
    let state_tx = channels.localization.state_tx.clone();
    // 러너블 실행 동안 재사용할 누적 상태 구조체.
    let mut runtime = LocalizationRuntime::new();

    loop {
        match imu_rx.recv().await {
            Ok(imu) => {
                // 방송 지연 시 최신 샘플만 사용하도록 큐를 비우고 가장 뒤 데이터를 사용한다.
                let mut newest = imu;
                while let Ok(newer) = imu_rx.try_recv() {
                    newest = newer;
                }

                // IMU 데이터를 라이브러리로 전달해 위치/자세를 계산하고,
                // 성공 시 즉시 Localization 채널로 브로드캐스트한다.
                match process_imu_sample(id, &scenario, start_coord, newest.as_ref(), &mut runtime)
                {
                    Ok(state) => {
                        let _ = state_tx.send(Arc::new(state));
                    }
                    Err(err) => {
                        eprintln!("[{}] Localization 처리 중 오류: {}", id, err);
                    }
                }
            }
            Err(RecvError::Lagged(skipped)) => {
                // 지속적으로 패킷이 밀리는 경우 현장 환경을 점검해야 한다.
                eprintln!(
                    "[{}] Localization IMU 채널이 {}개 프레임을 놓쳤습니다.",
                    id, skipped
                );
            }
            Err(RecvError::Closed) => {
                // IMU 공급 태스크가 종료되면 Localization도 정리한다.
                println!("[{}] IMU 채널이 닫혀 로컬라이제이션을 종료합니다.", id);
                break;
            }
        }
    }
}

/// Localization 결과를 구독해 도착지에 근접했는지 판정한다.
pub async fn runnable_adas_arrival(id: &'static str, channels: RteChannels) {
    let scenario = LOCALIZATION_ACTIVE_SCENARIO;
    let map = match MapData::load(scenario.map) {
        Ok(map) => map,
        Err(err) => {
            eprintln!(
                "[{}] Arrival detector: map({:?}) 로딩 실패: {}",
                id, scenario.map, err
            );
            return;
        }
    };

    let destination = match map.waypoint(
        scenario.destination.lane,
        scenario.destination.waypoint_index,
    ) {
        Some(coord) => [coord.x as f64, coord.y as f64],
        None => {
            eprintln!(
                "[{}] Arrival detector: 도착 waypoint_index={}가 맵 데이터 범위 밖입니다.",
                id, scenario.destination.waypoint_index
            );
            return;
        }
    };

    println!(
        "[{}] Arrival detector armed: map={:?}, lane={:?}, dest=({:.3}, {:.3}), threshold={:.1}cm",
        id,
        scenario.map,
        scenario.destination.lane,
        destination[0],
        destination[1],
        LOCALIZATION_ARRIVAL_THRESHOLD_M * 100.0
    );

    let mut state_rx = channels.localization.state_tx.subscribe();
    let arrival_tx = channels.localization.arrival_tx.clone();
    let mut arrival_reported = false;
    let mut alive_cnt: u32 = 0;

    loop {
        match state_rx.recv().await {
            Ok(state_arc) => {
                let state: &DtoLocalizationState = Arc::as_ref(&state_arc);
                let dx = state.position_map_xy[0] - destination[0];
                let dy = state.position_map_xy[1] - destination[1];
                let distance = (dx * dx + dy * dy).sqrt();

                let arrived = distance <= LOCALIZATION_ARRIVAL_THRESHOLD_M;
                if arrived {
                    if !arrival_reported {
                        println!(
                            "[{}] Arrival detected: distance={:.3}m, timestamp_ns={}",
                            id, distance, state.timestamp_ns
                        );
                        arrival_reported = true;
                    }
                } else if arrival_reported && distance > LOCALIZATION_ARRIVAL_THRESHOLD_M * 1.5 {
                    // 차량이 다시 멀어지면 재판정을 위해 재무장한다.
                    arrival_reported = false;
                }

                let dto = Arc::new(DtoLocalizationArrival::new(
                    arrived,
                    distance,
                    state.timestamp_ns,
                    alive_cnt,
                ));
                let _ = arrival_tx.send(dto);
                alive_cnt = alive_cnt.wrapping_add(1);
            }
            Err(RecvError::Lagged(skipped)) => {
                eprintln!(
                    "[{}] Arrival detector lagged by {} localization frames",
                    id, skipped
                );
            }
            Err(RecvError::Closed) => {
                println!("[{}] Arrival detector: localization 채널 종료", id);
                break;
            }
        }
    }
}
