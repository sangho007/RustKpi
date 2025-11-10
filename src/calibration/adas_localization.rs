//! ADAS Localization에서 사용할 지도·출발지·도착지 프리셋.
//! 지도 JSON 파일은 `src/asw/lib/` 아래에 있으며, 여기서 선택 가능한 기준점을 정의한다.

/// 지도 자산 식별자.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalizationMapId {
    /// 1자 직선 주행 코스.
    OneLane,
    /// S자 코스.
    SLane,
    /// 사거리(교차로) 코스.
    Crossroad,
}

impl LocalizationMapId {
    /// 지도 JSON 파일 경로를 반환한다.
    pub const fn json_asset(self) -> &'static str {
        match self {
            LocalizationMapId::OneLane => "src/asw/lib/map_data_1lane_quantized_chagable.json",
            LocalizationMapId::SLane => "src/asw/lib/map_data_slane_quantized_chagable.json",
            LocalizationMapId::Crossroad => "src/asw/lib/map_data_4lane_quantized_chagable.json",
        }
    }
}

/// 레인 종류.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocalizationLane {
    Inner,
    Outer,
}

/// 지도상의 좌표.
#[derive(Clone, Copy, Debug)]
pub struct MapCoordinate {
    pub x: f32,
    pub y: f32,
}

/// 출발지 옵션 정의.
#[derive(Clone, Copy, Debug)]
pub struct LocalizationStart {
    pub id: &'static str,
    pub display_name: &'static str,
    pub lane: LocalizationLane,
    pub waypoint_index: usize,
}

/// 도착지 옵션 정의.
#[derive(Clone, Copy, Debug)]
pub struct LocalizationDestination {
    pub id: &'static str,
    pub display_name: &'static str,
    pub lane: LocalizationLane,
    pub waypoint_index: usize,
    /// 이동 계획 유형(예: 좌회전, 직진). 필요 없으면 `None`.
    pub category: Option<&'static str>,
}

/// 지도별로 지원하는 출발/도착 프리셋 묶음.
#[derive(Clone, Copy, Debug)]
pub struct LocalizationMapPreset {
    pub map: LocalizationMapId,
    pub display_name: &'static str,
    pub starts: &'static [LocalizationStart],
    pub destinations: &'static [LocalizationDestination],
}

const ONE_LANE_STARTS: [LocalizationStart; 2] = [
    LocalizationStart {
        id: "start_inner",
        display_name: "출발 1차선",
        lane: LocalizationLane::Inner,
        waypoint_index: 44,
    },
    LocalizationStart {
        id: "start_outer",
        display_name: "출발 2차선",
        lane: LocalizationLane::Outer,
        waypoint_index: 44,
    },
];

const ONE_LANE_DESTINATIONS: [LocalizationDestination; 2] = [
    LocalizationDestination {
        id: "goal_inner",
        display_name: "도착 1차선",
        lane: LocalizationLane::Inner,
        waypoint_index: 0,
        category: None,
    },
    LocalizationDestination {
        id: "goal_outer",
        display_name: "도착 2차선",
        lane: LocalizationLane::Outer,
        waypoint_index: 0,
        category: None,
    },
];

const SLANE_STARTS: [LocalizationStart; 2] = [
    LocalizationStart {
        id: "start_inner",
        display_name: "출발 1차선",
        lane: LocalizationLane::Inner,
        waypoint_index: 56,
    },
    LocalizationStart {
        id: "start_outer",
        display_name: "출발 2차선",
        lane: LocalizationLane::Outer,
        waypoint_index: 58,
    },
];

const SLANE_DESTINATIONS: [LocalizationDestination; 2] = [
    LocalizationDestination {
        id: "goal_inner",
        display_name: "도착 1차선",
        lane: LocalizationLane::Inner,
        waypoint_index: 0,
        category: None,
    },
    LocalizationDestination {
        id: "goal_outer",
        display_name: "도착 2차선",
        lane: LocalizationLane::Outer,
        waypoint_index: 0,
        category: None,
    },
];

const CROSSROAD_STARTS: [LocalizationStart; 2] = [
    LocalizationStart {
        id: "start_inner",
        display_name: "출발 1차선",
        lane: LocalizationLane::Inner,
        waypoint_index: 94,
    },
    LocalizationStart {
        id: "start_outer",
        display_name: "출발 2차선",
        lane: LocalizationLane::Outer,
        waypoint_index: 79,
    },
];

const CROSSROAD_DESTINATIONS: [LocalizationDestination; 4] = [
    LocalizationDestination {
        id: "goal_inner_left",
        display_name: "도착 1차선 좌회전",
        lane: LocalizationLane::Inner,
        waypoint_index: 76,
        category: Some("좌회전"),
    },
    LocalizationDestination {
        id: "goal_outer_left",
        display_name: "도착 2차선 좌회전",
        lane: LocalizationLane::Outer,
        waypoint_index: 50,
        category: Some("좌회전"),
    },
    LocalizationDestination {
        id: "goal_inner_straight",
        display_name: "도착 1차선 직진",
        lane: LocalizationLane::Inner,
        waypoint_index: 0,
        category: Some("직진"),
    },
    LocalizationDestination {
        id: "goal_outer_straight",
        display_name: "도착 2차선 직진",
        lane: LocalizationLane::Outer,
        waypoint_index: 0,
        category: Some("직진"),
    },
];

/// 단일 시나리오 테스트용 지도/출발지/도착지 선택.
/// 필요한 값만 고쳐서 다른 맵 조합을 빠르게 검증한다.
#[derive(Clone, Copy, Debug)]
pub struct LocalizationScenarioSelection {
    pub map: LocalizationMapId,
    pub start: LocalizationStart,
    pub destination: LocalizationDestination,
}

/// 도착 판정을 위한 거리 임계값(미터). `LOCALIZATION_ACTIVE_SCENARIO` 기준으로 조정한다.
pub const LOCALIZATION_ARRIVAL_THRESHOLD_M: f64 = 0.10; // 10cm

// === 테스트할 시나리오 선택 영역 ===
// map/start/destination을 원하는 프리셋 값으로 바꿔서 실험한다.
/// 현재 테스트 시나리오 기본값.
pub const LOCALIZATION_ACTIVE_SCENARIO: LocalizationScenarioSelection =
    LocalizationScenarioSelection {
        // 1자 맵
        map: LocalizationMapId::OneLane,
        start: ONE_LANE_STARTS[0],
        destination: ONE_LANE_DESTINATIONS[0],
        // S자 맵
        //map: LocalizationMapId::SLane,
        //start: SLANE_STARTS[0],
        //destination: SLANE_DESTINATIONS[0],

        // 사거리 맵
        //map: LocalizationMapId::Crossroad,
        //start: CROSSROAD_STARTS[1],
        //destination: CROSSROAD_DESTINATIONS[3],
    };

/// ADAS Localization에서 선택 가능한 지도 프리셋.
pub const LOCALIZATION_MAP_PRESETS: &[LocalizationMapPreset] = &[
    LocalizationMapPreset {
        map: LocalizationMapId::OneLane,
        display_name: "1자 맵",
        starts: &ONE_LANE_STARTS,
        destinations: &ONE_LANE_DESTINATIONS,
    },
    LocalizationMapPreset {
        map: LocalizationMapId::SLane,
        display_name: "S자 맵",
        starts: &SLANE_STARTS,
        destinations: &SLANE_DESTINATIONS,
    },
    LocalizationMapPreset {
        map: LocalizationMapId::Crossroad,
        display_name: "사거리 맵",
        starts: &CROSSROAD_STARTS,
        destinations: &CROSSROAD_DESTINATIONS,
    },
];
