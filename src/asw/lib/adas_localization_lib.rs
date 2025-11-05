//! ADAS Localization 공용 라이브러리.
//! 러너블이 복잡한 좌표 변환/센서 융합 로직을 직접 구현하지 않도록
//! - 지도 JSON 파싱
//! - IMU 기반 변위 누적
//! - 차량 yaw 축 판별 및 보정
//! 을 캡슐화한 헬퍼 함수들을 제공한다.

use crate::calibration::adas_localization::{
    LocalizationLane, LocalizationMapId, LocalizationScenarioSelection, MapCoordinate,
};
use crate::rte::rte_dto::{DtoImu, DtoLocalizationState, LocalizationYawSource};
use serde::Deserialize;
use std::f64::consts::PI;
use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

pub const MOTION_HEADING_MIN_STEP_M: f64 = 0.01; // 1cm 이상 움직였을 때만 헤딩 계산.
pub const AXIS_SELECTION_MAX_ERROR_RAD: f64 = PI / 4.0; // 45도 이내면 동일 축으로 간주.

/// ADAS Localization에서 반복적으로 사용하는 런타임 상태 묶음.
pub struct LocalizationRuntime {
    /// IMU 절대 좌표의 기준점(초기 위치). None이면 아직 설정되지 않음.
    pub base_imu_position: Option<[f64; 3]>,
    /// 직전 계산된 맵 좌표. 누적 변위가 충분할 때만 heading 계산에 활용한다.
    pub last_map_position: Option<[f64; 2]>,
    /// 가장 최근 yaw 결과와 데이터 출처(센서, 추정 등). 센서 공백 시 fallback 용도.
    pub last_yaw: Option<(f64, LocalizationYawSource)>,
    /// IMU 오일러 각 중 차량 yaw에 대응하는 축. 움직임과 비교해 한 번 선택하면 유지한다.
    pub yaw_axis: Option<OrientationAxis>,
    /// 선택된 축과 실제 차량 yaw 사이의 편차(rad). 축 결정 시 함께 저장해 보정한다.
    pub yaw_bias: f64,
    /// 주기적인 디버그 로그 출력을 위한 타임스탬프.
    pub last_log: Instant,
}

impl LocalizationRuntime {
    pub fn new() -> Self {
        Self {
            base_imu_position: None,
            last_map_position: None,
            last_yaw: None,
            yaw_axis: None,
            yaw_bias: 0.0,
            last_log: Instant::now(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum OrientationAxis {
    Yaw,
    Pitch,
    Roll,
}

impl OrientationAxis {
    fn extract(self, orientation: &[f64; 3]) -> f64 {
        // yaw-roll-pitch 배열에서 축에 해당하는 원소만 뽑아낸다.
        match self {
            OrientationAxis::Yaw => orientation[0],
            OrientationAxis::Roll => orientation[1],
            OrientationAxis::Pitch => orientation[2],
        }
    }

    fn to_source(self) -> LocalizationYawSource {
        match self {
            OrientationAxis::Yaw => LocalizationYawSource::ImuYaw,
            OrientationAxis::Pitch => LocalizationYawSource::ImuPitch,
            OrientationAxis::Roll => LocalizationYawSource::ImuRoll,
        }
    }
}

#[derive(Debug)]
pub struct MapData {
    inner: Vec<MapCoordinate>,
    outer: Vec<MapCoordinate>,
}

impl MapData {
    pub fn load(map_id: LocalizationMapId) -> Result<Self, String> {
        // 캘리브레이션에서 지정한 지도 JSON 하나를 읽어 inner/outer 레인을 모두 확보한다.
        let path = Path::new(map_id.json_asset());
        let text = fs::read_to_string(path)
            .map_err(|err| format!("{} 읽기 실패: {}", path.display(), err))?;
        let raw: RawMap = serde_json::from_str(&text)
            .map_err(|err| format!("{} 파싱 실패: {}", path.display(), err))?;
        let inner = raw
            .inner_waypoint
            .into_iter()
            .map(MapCoordinate::from)
            .collect();
        let outer = raw
            .outer_waypoint
            .into_iter()
            .map(MapCoordinate::from)
            .collect();
        Ok(Self { inner, outer })
    }

    pub fn waypoint(&self, lane: LocalizationLane, index: usize) -> Option<MapCoordinate> {
        match lane {
            LocalizationLane::Inner => self.inner.get(index),
            LocalizationLane::Outer => self.outer.get(index),
        }
        .copied()
    }
}

#[derive(Debug, Deserialize)]
struct RawMap {
    #[serde(default)]
    inner_waypoint: Vec<RawWaypoint>,
    #[serde(default)]
    outer_waypoint: Vec<RawWaypoint>,
}

#[derive(Debug, Deserialize)]
struct RawWaypoint {
    position: [f32; 2],
    #[serde(default)]
    _can_change_lane: bool,
}

impl From<RawWaypoint> for MapCoordinate {
    fn from(raw: RawWaypoint) -> Self {
        let x = raw.position.get(0).copied().unwrap_or_default();
        let y = raw.position.get(1).copied().unwrap_or_default();
        MapCoordinate { x, y }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn process_imu_sample(
    id: &str,
    scenario: &LocalizationScenarioSelection,
    start_coord: MapCoordinate,
    imu: &DtoImu,
    runtime: &mut LocalizationRuntime,
) -> Result<DtoLocalizationState, &'static str> {
    let pose = imu
        .pose
        .as_ref()
        .ok_or("IMU pose 데이터가 없습니다 (position_world 없음)")?;
    let imu_position = pose
        .position_world
        .ok_or("IMU pose.position_world가 비어 있습니다")?;

    if runtime.base_imu_position.is_none() {
        // 최초 한 번만 IMU의 절대 위치를 기준점으로 저장한다.
        runtime.base_imu_position = Some(imu_position);
        println!(
            "[{}] Localization 기준점 IMU 위치를 초기화했습니다: ({:.3}, {:.3}, {:.3})",
            id, imu_position[0], imu_position[1], imu_position[2]
        );
    }
    let base = runtime.base_imu_position.unwrap();
    let displacement = [
        imu_position[0] - base[0],
        imu_position[1] - base[1],
        imu_position[2] - base[2],
    ];

    let map_position = [
        start_coord.x as f64 + displacement[0],
        start_coord.y as f64 - displacement[2],
    ];

    let motion_heading = runtime
        .last_map_position
        .map(|prev| {
            // 이전 맵 좌표와의 차이를 이용해 주행 방향 벡터를 구한다.
            let delta = [map_position[0] - prev[0], map_position[1] - prev[1]];
            let dist_sq = delta[0] * delta[0] + delta[1] * delta[1];
            if dist_sq.sqrt() >= MOTION_HEADING_MIN_STEP_M {
                Some(delta[1].atan2(delta[0]))
            } else {
                None
            }
        })
        .flatten();

    let mut yaw_candidate: Option<(f64, LocalizationYawSource)> = None;
    if let Some(orientation) = pose.orientation_yaw_roll_pitch.as_ref() {
        if let Some(axis) = runtime.yaw_axis {
            // 이미 축이 선택되어 있으면 해당 축과 bias로 yaw를 바로 계산한다.
            let raw = axis.extract(orientation);
            yaw_candidate = Some((wrap_angle(raw + runtime.yaw_bias), axis.to_source()));
        } else if let Some(heading) = motion_heading {
            // 아직 축이 없다면 이동 방향과 가장 가까운 오일러 축을 찾는다.
            let candidates = [
                (OrientationAxis::Yaw, orientation[0]),
                (OrientationAxis::Pitch, orientation[2]),
                (OrientationAxis::Roll, orientation[1]),
            ];
            let mut best: Option<(OrientationAxis, f64, f64)> = None;
            for (axis, raw) in candidates {
                let diff = wrap_angle(heading - raw);
                if best
                    .map(|(_, _, best_err)| diff.abs() < best_err.abs())
                    .unwrap_or(true)
                {
                    best = Some((axis, raw, diff));
                }
            }
            if let Some((axis, raw, diff)) = best {
                if diff.abs() <= AXIS_SELECTION_MAX_ERROR_RAD {
                    runtime.yaw_axis = Some(axis);
                    runtime.yaw_bias = diff;
                    println!(
                        "[{}] Localization yaw 축을 {:?}로 선택 (bias {:.2} deg)",
                        id,
                        axis,
                        rad_to_deg(runtime.yaw_bias)
                    );
                    // 선택된 축과 편차를 이용해 보정된 yaw를 기록한다.
                    yaw_candidate = Some((wrap_angle(raw + runtime.yaw_bias), axis.to_source()));
                }
            }
        }
    }

    if yaw_candidate.is_none() {
        if let Some(heading) = motion_heading {
            // 1순위: 충분히 이동했다면 heading을 yaw로 사용한다.
            yaw_candidate = Some((wrap_angle(heading), LocalizationYawSource::MotionEstimate));
        } else if let Some((prev_yaw, prev_src)) = runtime.last_yaw.as_ref() {
            // 2순위: 직전 yaw를 유지한다.
            yaw_candidate = Some((*prev_yaw, *prev_src));
        } else if let Some(orientation) = pose.orientation_yaw_roll_pitch.as_ref() {
            // 3순위: 센서가 yaw 축을 직접 제공한다고 가정한다.
            let raw = orientation[0];
            yaw_candidate = Some((wrap_angle(raw), LocalizationYawSource::ImuYaw));
        }
    }

    let (yaw_rad, yaw_source) = yaw_candidate.ok_or("yaw 추정에 실패했습니다")?;
    runtime.last_yaw = Some((yaw_rad, yaw_source));
    runtime.last_map_position = Some(map_position);

    let state = DtoLocalizationState::new(
        scenario.map,
        scenario.start.lane,
        map_position,
        displacement,
        yaw_rad,
        yaw_source,
        motion_heading.map(wrap_angle),
        imu.header.stamp_ns,
        imu.alive_cnt,
    );

    if runtime.last_log.elapsed() >= Duration::from_millis(500) {
        // 주기별로 핵심 값을 출력해 센서 데이터 흐름을 확인한다.
        println!(
            "[{}] pose=({:.3}, {:.3}) yaw={:.2} deg (src={:?}) motion={}",
            id,
            map_position[0],
            map_position[1],
            rad_to_deg(yaw_rad),
            yaw_source,
            motion_heading
                .map(|h| format!("{:.2} deg", rad_to_deg(h)))
                .unwrap_or_else(|| "--".to_string())
        );
        runtime.last_log = Instant::now();
    }

    Ok(state)
}

pub fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

pub fn rad_to_deg(rad: f64) -> f64 {
    rad.to_degrees()
}
