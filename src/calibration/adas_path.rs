//! ADAS 경로 계획(전역/지역) 관련 캘리브레이션 값.
//! 경로 탐색 알고리즘에서 사용하는 하드코딩된 상수를 분리해 조정 가능하도록 한다.

use std::time::Duration;

/// 전역 경로 탐색 시 사용할 알고리즘.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GlobalPathPlanner {
    /// 기존 격자 기반 A*.
    AStar,
    /// 차량 헤딩을 상태에 포함하는 하이브리드 A*.
    HybridAStar,
}

/// 전역 경로 탐색 매개변수 묶음.
#[derive(Clone, Debug)]
pub struct AdasPathGlobalCalibration {
    /// 경로 재계획 주기.
    pub replanning_period: Duration,
    /// 차량 차폭(m).
    pub vehicle_width_m: f32,
    /// 차량 길이(m). 현재 알고리즘에서는 여유 거리 계산에만 사용한다.
    pub vehicle_length_m: f32,
    /// 차선 변경 시 허용할 최대 횡방향 오프셋(m).
    pub max_lane_change_offset_m: f32,
    /// 동일 차선에서 후진을 허용하는 여유 거리(m).
    pub forward_tolerance_m: f32,
    /// 차선 변경 시 부과할 추가 비용(m).
    pub lane_change_penalty_m: f32,
    /// 강제 차선 변경 모드에서 사용할 추가 비용(m). 기본값은 0으로 동일 차선보다 우선되도록 한다.
    pub forced_lane_change_penalty_m: f32,
    /// 탐색 중 허용되는 최대 차선 변경 횟수.
    pub max_lane_changes: u32,
    /// 강제 차선 변경 모드에서 허용하는 최대 차선 변경 횟수.
    pub forced_max_lane_changes: u32,
    /// 동일 차선 내 후보 이웃 개수.
    pub same_lane_neighbors: usize,
    /// 차선 변경 시 고려할 후보 개수.
    pub cross_lane_neighbors: usize,
    /// 동일 차선 내 연결을 허용할 최대 거리(m).
    pub max_same_lane_distance_m: f32,
    /// 차선 변경 후보 검색 한계. 필요 시 후보를 줄이기 위해 사용한다.
    pub max_lane_change_candidates: usize,
    /// 시작점 탐색 시 사용될 최대 이웃 waypoint 개수.
    pub nearest_search_horizon: usize,
    /// 시작 waypoint 후보에서 헤딩과의 내적이 이 값 이상인 항목을 우선한다.
    pub nearest_heading_cos_threshold: f64,
    /// 장애물 앞 waypoint를 차단할 때 적용할 여유 거리(m).
    pub obstacle_block_margin_m: f32,
    /// 차단된 waypoint를 유지할 시간.
    pub obstacle_block_timeout: Duration,
    /// 장애물 차단 시 차량 헤딩과 비교할 허용 각도(도 단위).
    pub obstacle_block_heading_tolerance_deg: f32,
    /// 장애물 차단을 시작하기 전에 건너뛸 거리(m).
    pub obstacle_block_start_offset_m: f32,
    /// lane change 완료로 판단할 최대 횡방향 오차(m).
    pub lane_change_completion_tolerance_m: f32,
    /// lane change 완료 판정 시 허용할 yaw 차이(도 단위).
    pub lane_change_completion_heading_tolerance_deg: f32,
    /// 차선 변경 불가 구간을 얼마나 미리 감지할지에 대한 거리(m).
    pub lane_change_forbidden_lookahead_m: f32,
    /// 불가 구간 임박 시 lane change 페널티에 곱할 비율.
    pub lane_change_forbidden_penalty_scale: f32,
    /// 전역 경로 탐색에 사용할 알고리즘.
    pub global_planner: GlobalPathPlanner,
    /// 강제 차선 변경 모드에서 첫 lane change 전 동일 차선 간선에 더할 비용.
    pub forced_same_lane_penalty_m: f32,
}

impl Default for AdasPathGlobalCalibration {
    fn default() -> Self {
        Self {
            replanning_period: Duration::from_millis(2000),
            vehicle_width_m: 0.15,
            vehicle_length_m: 0.20,
            max_lane_change_offset_m: 1.0,
            forward_tolerance_m: 0.02,
            lane_change_penalty_m: 1.0,
            forced_lane_change_penalty_m: -0.5,
            max_lane_changes: 5,
            forced_max_lane_changes: 5,
            same_lane_neighbors: 4,
            cross_lane_neighbors: 5,
            max_same_lane_distance_m: 0.1,
            max_lane_change_candidates: 8,
            nearest_search_horizon: 12,
            nearest_heading_cos_threshold: 0.0,
            obstacle_block_margin_m: 0.0,
            obstacle_block_timeout: Duration::from_millis(7000),
            obstacle_block_heading_tolerance_deg: 45.0,
            obstacle_block_start_offset_m: 0.2,
            lane_change_completion_tolerance_m: 0.05,
            lane_change_completion_heading_tolerance_deg: 30.0,
            lane_change_forbidden_lookahead_m: 0.5,
            lane_change_forbidden_penalty_scale: 0.4,
            global_planner: GlobalPathPlanner::HybridAStar,
            forced_same_lane_penalty_m: 0.25,
            //global_planner: GlobalPathPlanner::AStar,
        }
    }
}

/// 단기 경로(로컬) 생성 시 필요한 매개변수.
#[derive(Clone, Debug)]
pub struct AdasPathLocalCalibration {
    /// 로컬 경로로 사용할 waypoint 개수.
    pub waypoint_window: usize,
    /// 스무딩된 경로로 생성할 샘플 수.
    pub smoothing_sample_count: usize,
    /// 스무딩 대상에서 제외할 선행 waypoint 수.
    pub smoothing_skip_head: usize,
    /// 스무딩을 수행하기 위한 최소 waypoint 수.
    pub smoothing_min_samples: usize,
}

impl Default for AdasPathLocalCalibration {
    fn default() -> Self {
        Self {
            waypoint_window: 7,
            smoothing_sample_count: 20,
            smoothing_skip_head: 0,
            smoothing_min_samples: 4,
        }
    }
}
