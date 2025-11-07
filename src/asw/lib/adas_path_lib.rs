//! ADAS 전역 경로 계획을 위한 공용 라이브러리.
//! Python A* 구현(`tools/waypoint_graph_path_plot.py`)을 참조해 러스트로 포팅한 버전이다.

use crate::calibration::{
    AdasPathGlobalCalibration, AdasPathLocalCalibration, GlobalPathPlanner, LocalizationLane,
    LocalizationMapId,
};
use crate::rte::rte_dto::{
    DtoAdasGlobalPath, DtoAdasLocalPath, DtoAdasSmoothedPath, DtoLocalizationState, DtoPathWaypoint,
};
use serde::Deserialize;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f64::consts::PI;
use std::fs;
use std::hash::{Hash, Hasher};
use std::path::Path;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use tokio::sync::broadcast;

/// 경로 계획에 사용되는 waypoint 표현.
#[derive(Clone, Debug)]
pub struct PathWaypoint {
    pub lane: LocalizationLane,
    pub index: usize,
    pub x: f64,
    pub y: f64,
    pub can_change_lane: bool,
}

impl PathWaypoint {
    pub fn position(&self) -> [f64; 2] {
        [self.x, self.y]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeKey {
    pub lane: LocalizationLane,
    pub index: usize,
}

impl Hash for NodeKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // LocalizationLane는 단순 열거형이므로 discriminant를 해시에 사용한다.
        match self.lane {
            LocalizationLane::Inner => 0u8.hash(state),
            LocalizationLane::Outer => 1u8.hash(state),
        }
        self.index.hash(state);
    }
}

#[derive(Clone, Debug)]
struct Neighbor {
    node: NodeKey,
    distance: f64,
    lane_change: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum PathPlanningMode {
    Normal,
    ForceLaneChange,
}

#[derive(Debug, Clone)]
pub struct PlannedPath {
    pub waypoints: Vec<PathWaypoint>,
    pub lane_change_count: u32,
}

#[derive(Clone, Debug)]
pub struct PathGraph {
    map_id: LocalizationMapId,
    inner: Vec<PathWaypoint>,
    outer: Vec<PathWaypoint>,
    adjacency: HashMap<NodeKey, Vec<Neighbor>>,
}

impl PathGraph {
    pub fn plan_path(
        &self,
        algorithm: GlobalPathPlanner,
        start: NodeKey,
        goal: NodeKey,
        blocked: &HashSet<NodeKey>,
        lane_change_penalty: f64,
        max_lane_changes: u32,
        forbidden_lookahead_m: f64,
        forbidden_penalty_scale: f64,
    ) -> Result<PlannedPath, String> {
        self.validate_plan_request(start, goal, blocked)?;

        match algorithm {
            GlobalPathPlanner::AStar => {
                self.plan_path_astar(
                    start,
                    goal,
                    blocked,
                    lane_change_penalty,
                    max_lane_changes,
                    forbidden_lookahead_m,
                    forbidden_penalty_scale,
                )
            }
            GlobalPathPlanner::HybridAStar => {
                self.plan_path_hybrid(
                    start,
                    goal,
                    blocked,
                    lane_change_penalty,
                    max_lane_changes,
                    forbidden_lookahead_m,
                    forbidden_penalty_scale,
                )
            }
        }
    }

    fn validate_plan_request(
        &self,
        start: NodeKey,
        goal: NodeKey,
        blocked: &HashSet<NodeKey>,
    ) -> Result<(), String> {
        if self.waypoint(start).is_none() {
            return Err(format!(
                "시작 waypoint가 지도에 없습니다: {:?}:{:?}",
                start.lane, start.index
            ));
        }
        if self.waypoint(goal).is_none() {
            return Err(format!(
                "목적지 waypoint가 지도에 없습니다: {:?}:{:?}",
                goal.lane, goal.index
            ));
        }
        if blocked.contains(&goal) {
            return Err("목적지 waypoint가 장애물로 차단되었습니다.".to_string());
        }
        Ok(())
    }

    pub fn load(
        map_id: LocalizationMapId,
        calib: &AdasPathGlobalCalibration,
    ) -> Result<Self, String> {
        let path = Path::new(map_id.json_asset());
        let text = fs::read_to_string(path)
            .map_err(|err| format!("{} 읽기 실패: {}", path.display(), err))?;
        let raw: RawMap = serde_json::from_str(&text)
            .map_err(|err| format!("{} 파싱 실패: {}", path.display(), err))?;

        let inner = raw
            .inner_waypoint
            .into_iter()
            .enumerate()
            .map(|(idx, wp)| PathWaypoint {
                lane: LocalizationLane::Inner,
                index: idx,
                x: wp.position[0] as f64,
                y: wp.position[1] as f64,
                can_change_lane: wp.can_change_lane,
            })
            .collect::<Vec<_>>();

        let outer = raw
            .outer_waypoint
            .into_iter()
            .enumerate()
            .map(|(idx, wp)| PathWaypoint {
                lane: LocalizationLane::Outer,
                index: idx,
                x: wp.position[0] as f64,
                y: wp.position[1] as f64,
                can_change_lane: wp.can_change_lane,
            })
            .collect::<Vec<_>>();

        let mut graph = Self {
            map_id,
            inner,
            outer,
            adjacency: HashMap::new(),
        };
        graph.build_edges(calib);
        Ok(graph)
    }

    pub fn map_id(&self) -> LocalizationMapId {
        self.map_id
    }

    pub fn waypoint(&self, key: NodeKey) -> Option<&PathWaypoint> {
        match key.lane {
            LocalizationLane::Inner => self.inner.get(key.index),
            LocalizationLane::Outer => self.outer.get(key.index),
        }
    }

    pub fn nearest_waypoint(
        &self,
        lane: LocalizationLane,
        position: [f64; 2],
        heading: Option<[f64; 2]>,
        horizon: usize,
        heading_cos_threshold: f64,
    ) -> Option<NodeKey> {
        let candidates = match lane {
            LocalizationLane::Inner => &self.inner,
            LocalizationLane::Outer => &self.outer,
        };
        if candidates.is_empty() {
            return None;
        }

        let mut scored = candidates
            .iter()
            .enumerate()
            .map(|(idx, wp)| {
                let dist = distance(position, wp.position());
                (dist, idx)
            })
            .collect::<Vec<_>>();

        scored.sort_by(|a, b| cmp_f64(a.0, b.0));
        let limit = horizon.max(1);
        let heading_unit = heading.and_then(|vec| {
            let norm = (vec[0] * vec[0] + vec[1] * vec[1]).sqrt();
            if norm <= f64::EPSILON {
                None
            } else {
                Some([vec[0] / norm, vec[1] / norm])
            }
        });
        let threshold = heading_cos_threshold.clamp(-1.0, 1.0);
        let mut fallback: Option<NodeKey> = None;

        for (_, idx) in scored.into_iter().take(limit) {
            let key = NodeKey { lane, index: idx };
            if fallback.is_none() {
                fallback = Some(key);
            }

            match heading_unit {
                Some(heading_vec) => {
                    let candidate = &candidates[idx];
                    let dir = [candidate.x - position[0], candidate.y - position[1]];
                    let norm = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
                    if norm <= f64::EPSILON {
                        return Some(key);
                    }
                    let dir_unit = [dir[0] / norm, dir[1] / norm];
                    let dot = dir_unit[0] * heading_vec[0] + dir_unit[1] * heading_vec[1];
                    if dot >= threshold {
                        return Some(key);
                    }
                }
                None => return Some(key),
            }
        }

        fallback
    }

    fn plan_path_astar(
        &self,
        start: NodeKey,
        goal: NodeKey,
        blocked: &HashSet<NodeKey>,
        lane_change_penalty: f64,
        max_lane_changes: u32,
        forbidden_lookahead_m: f64,
        forbidden_penalty_scale: f64,
    ) -> Result<PlannedPath, String> {
        let mut open = BinaryHeap::new();
        let mut g_score: HashMap<(NodeKey, u32), f64> = HashMap::new();
        let mut parent: HashMap<(NodeKey, u32), (NodeKey, u32)> = HashMap::new();

        let start_state = (start, 0u32);
        g_score.insert(start_state, 0.0);
        open.push(HeapNode::new(0.0, heuristic(self, start, goal), start, 0));

        while let Some(node) = open.pop() {
            if blocked.contains(&node.key) && node.key != start {
                continue;
            }
            if node.key == goal {
                let waypoints = reconstruct_path(self, &parent, (node.key, node.lane_changes));
                let lane_change_count = waypoints
                    .windows(2)
                    .filter(|pair| pair[0].lane != pair[1].lane)
                    .count() as u32;
                return Ok(PlannedPath {
                    waypoints,
                    lane_change_count,
                });
            }

            let neighbors = match self.adjacency.get(&node.key) {
                Some(list) => list,
                None => continue,
            };

            for neighbor in neighbors {
                if blocked.contains(&neighbor.node) {
                    continue;
                }
                let additional_change = if neighbor.node.lane != node.key.lane {
                    1
                } else {
                    0
                };
                let next_changes = node.lane_changes + additional_change;
                if next_changes > max_lane_changes {
                    continue;
                }

                let lane_penalty = if neighbor.lane_change {
                    let mut penalty = lane_change_penalty;
                    if forbidden_penalty_scale != 1.0
                        && self.has_imminent_lane_change_block(
                            node.key,
                            forbidden_lookahead_m,
                        )
                    {
                        penalty *= forbidden_penalty_scale;
                    }
                    penalty
                } else {
                    0.0
                };
                let step_cost = neighbor.distance + lane_penalty;
                let tentative = node.cost + step_cost;
                let neighbor_state = (neighbor.node, next_changes);
                let entry = g_score.entry(neighbor_state).or_insert(f64::INFINITY);
                if tentative + f64::EPSILON < *entry {
                    *entry = tentative;
                    parent.insert(neighbor_state, (node.key, node.lane_changes));
                    open.push(HeapNode::new(
                        tentative,
                        tentative + heuristic(self, neighbor.node, goal),
                        neighbor.node,
                        next_changes,
                    ));
                }
            }
        }

        Err("경로를 찾을 수 없습니다.".to_string())
    }

    fn primary_heading(&self, node: NodeKey) -> Option<f64> {
        let current_wp = self.waypoint(node)?;
        let current_pos = current_wp.position();
        let neighbors = self.adjacency.get(&node)?;

        let mut best_heading: Option<f64> = None;
        let mut best_same_lane = false;
        let mut best_dist = f64::INFINITY;

        for neighbor in neighbors {
            if let Some(target_wp) = self.waypoint(neighbor.node) {
                let heading = heading_between(current_pos, target_wp.position());
                let same_lane = neighbor.node.lane == node.lane;

                let is_better = match best_heading {
                    None => true,
                    Some(_) => {
                        if same_lane && !best_same_lane {
                            true
                        } else if same_lane == best_same_lane {
                            neighbor.distance + f64::EPSILON < best_dist
                        } else {
                            false
                        }
                    }
                };

                if is_better {
                    best_heading = Some(heading);
                    best_same_lane = same_lane;
                    best_dist = neighbor.distance;
                }
            }
        }

        best_heading
    }

    fn heading_candidates(&self, node: NodeKey, goal: NodeKey, bins: u16) -> Vec<u16> {
        let Some(origin_wp) = self.waypoint(node) else {
            return vec![0];
        };
        let origin_pos = origin_wp.position();
        let mut result: Vec<u16> = Vec::new();

        if let Some(primary) = self.primary_heading(node) {
            let idx = heading_to_index(primary, bins);
            result.push(idx);
        }

        if let Some(neighbors) = self.adjacency.get(&node) {
            for neighbor in neighbors.iter().take(6) {
                if let Some(target_wp) = self.waypoint(neighbor.node) {
                    let heading = heading_between(origin_pos, target_wp.position());
                    let idx = heading_to_index(heading, bins);
                    if !result.iter().any(|&cand| cand == idx) {
                        result.push(idx);
                    }
                }
            }
        }

        if let Some(goal_wp) = self.waypoint(goal) {
            let heading = heading_between(origin_pos, goal_wp.position());
            let idx = heading_to_index(heading, bins);
            if !result.iter().any(|&cand| cand == idx) {
                result.push(idx);
            }
        }

        if result.is_empty() {
            result.push(0);
        }
        result
    }

    fn plan_path_hybrid(
        &self,
        start: NodeKey,
        goal: NodeKey,
        blocked: &HashSet<NodeKey>,
        lane_change_penalty: f64,
        max_lane_changes: u32,
        forbidden_lookahead_m: f64,
        forbidden_penalty_scale: f64,
    ) -> Result<PlannedPath, String> {
        const HEADING_BINS: u16 = 36;
        const TURN_PENALTY_COEFF: f64 = 2.0; // 1.5
        const HEADING_HEURISTIC_WEIGHT: f64 = 0.75; // 0.75
        const MAX_BASE_HEADING_DELTA: f64 = PI / 2.0;
        const MAX_LANE_CHANGE_DELTA: f64 = PI * 0.5;

        let goal_heading_idx = {
            let goal_heading = self
                .primary_heading(goal)
                .or_else(|| {
                    self.waypoint(goal)
                        .zip(self.waypoint(start))
                        .map(|(goal_wp, start_wp)| {
                            heading_between(goal_wp.position(), start_wp.position())
                        })
                })
                .unwrap_or(0.0);
            heading_to_index(goal_heading, HEADING_BINS)
        };

        let mut open = BinaryHeap::new();
        let mut g_score: HashMap<(NodeKey, u16, u32), f64> = HashMap::new();
        let mut parent: HashMap<(NodeKey, u16, u32), (NodeKey, u16, u32)> = HashMap::new();

        let start_headings = self.heading_candidates(start, goal, HEADING_BINS);
        for heading_idx in start_headings {
            let state = (start, heading_idx, 0u32);
            g_score.insert(state, 0.0);
            let estimate = hybrid_heuristic(
                self,
                start,
                heading_idx,
                goal,
                goal_heading_idx,
                HEADING_BINS,
                HEADING_HEURISTIC_WEIGHT,
            );
            open.push(HybridHeapNode::new(0.0, estimate, start, heading_idx, 0));
        }

        while let Some(node) = open.pop() {
            if blocked.contains(&node.key) && node.key != start {
                continue;
            }
            if node.key == goal {
                let waypoints = reconstruct_path_hybrid(
                    self,
                    &parent,
                    (node.key, node.heading_idx, node.lane_changes),
                );
                let lane_change_count = waypoints
                    .windows(2)
                    .filter(|pair| pair[0].lane != pair[1].lane)
                    .count() as u32;
                return Ok(PlannedPath {
                    waypoints,
                    lane_change_count,
                });
            }

            let neighbors = match self.adjacency.get(&node.key) {
                Some(list) => list,
                None => continue,
            };

            let current_heading = index_to_heading(node.heading_idx, HEADING_BINS);

            for neighbor in neighbors {
                if blocked.contains(&neighbor.node) {
                    continue;
                }

                let additional_change = if neighbor.node.lane != node.key.lane {
                    1
                } else {
                    0
                };
                let next_changes = node.lane_changes + additional_change;
                if next_changes > max_lane_changes {
                    continue;
                }

                let Some(current_wp) = self.waypoint(node.key) else {
                    continue;
                };
                let Some(target_wp) = self.waypoint(neighbor.node) else {
                    continue;
                };

                let next_heading = heading_between(current_wp.position(), target_wp.position());
                let heading_delta = heading_difference(next_heading, current_heading);

                let max_delta = if neighbor.lane_change {
                    MAX_LANE_CHANGE_DELTA
                } else {
                    MAX_BASE_HEADING_DELTA
                };
                if heading_delta.abs() > max_delta {
                    continue;
                }

                let curvature_penalty = heading_delta.abs() * TURN_PENALTY_COEFF;
                let lane_penalty = if neighbor.lane_change {
                    let mut penalty = lane_change_penalty;
                    if forbidden_penalty_scale != 1.0
                        && self.has_imminent_lane_change_block(
                            node.key,
                            forbidden_lookahead_m,
                        )
                    {
                        penalty *= forbidden_penalty_scale;
                    }
                    penalty
                } else {
                    0.0
                };

                let step_cost = neighbor.distance + lane_penalty + curvature_penalty;
                let tentative = node.cost + step_cost;

                let heading_idx = heading_to_index(next_heading, HEADING_BINS);
                let neighbor_state = (neighbor.node, heading_idx, next_changes);
                let entry = g_score.entry(neighbor_state).or_insert(f64::INFINITY);

                if tentative + f64::EPSILON < *entry {
                    *entry = tentative;
                    parent.insert(
                        neighbor_state,
                        (node.key, node.heading_idx, node.lane_changes),
                    );

                    let estimate = tentative
                        + hybrid_heuristic(
                            self,
                            neighbor.node,
                            heading_idx,
                            goal,
                            goal_heading_idx,
                            HEADING_BINS,
                            HEADING_HEURISTIC_WEIGHT,
                        );
                    open.push(HybridHeapNode::new(
                        tentative,
                        estimate,
                        neighbor.node,
                        heading_idx,
                        next_changes,
                    ));
                }
            }
        }

        Err("경로를 찾을 수 없습니다.".to_string())
    }

    fn build_edges(&mut self, calib: &AdasPathGlobalCalibration) {
        self.build_same_lane_edges(LocalizationLane::Inner, calib);
        self.build_same_lane_edges(LocalizationLane::Outer, calib);
        self.connect_lanes(LocalizationLane::Inner, LocalizationLane::Outer, calib);
        self.connect_lanes(LocalizationLane::Outer, LocalizationLane::Inner, calib);
    }

    fn lane_points(&self, lane: LocalizationLane) -> &[PathWaypoint] {
        match lane {
            LocalizationLane::Inner => &self.inner,
            LocalizationLane::Outer => &self.outer,
        }
    }

    fn has_imminent_lane_change_block(
        &self,
        node: NodeKey,
        lookahead_m: f64,
    ) -> bool {
        if lookahead_m <= f64::EPSILON {
            return false;
        }
        let lane_points = self.lane_points(node.lane);
        let Some(current_wp) = lane_points.get(node.index) else {
            return false;
        };
        let mut prev = current_wp.position();
        let mut accumulated = 0.0;
        for wp in lane_points.iter().skip(node.index + 1) {
            let pos = wp.position();
            accumulated += distance(prev, pos);
            prev = pos;
            if accumulated > lookahead_m {
                break;
            }
            if !wp.can_change_lane {
                return true;
            }
        }
        false
    }

    fn build_same_lane_edges(&mut self, lane: LocalizationLane, calib: &AdasPathGlobalCalibration) {
        let lane_points = self.lane_points(lane).to_vec();
        for current in &lane_points {
            let mut candidates: Vec<(f64, usize)> = Vec::new();
            for other in &lane_points {
                if current.index == other.index {
                    continue;
                }
                if other.y + (calib.forward_tolerance_m as f64) < current.y {
                    continue;
                }
                let dist = distance(current.position(), other.position());
                if dist > calib.max_same_lane_distance_m as f64 {
                    continue;
                }
                candidates.push((dist, other.index));
            }
            candidates.sort_by(|a, b| cmp_f64(a.0, b.0));
            for (dist, target_idx) in candidates.into_iter().take(calib.same_lane_neighbors) {
                self.add_edge(
                    NodeKey {
                        lane,
                        index: current.index,
                    },
                    NodeKey {
                        lane,
                        index: target_idx,
                    },
                    dist,
                    false,
                );
            }
        }
    }

    fn connect_lanes(
        &mut self,
        source_lane: LocalizationLane,
        target_lane: LocalizationLane,
        calib: &AdasPathGlobalCalibration,
    ) {
        let source_pts = self.lane_points(source_lane).to_vec();
        let target_pts = self.lane_points(target_lane).to_vec();

        for src in &source_pts {
            if !src.can_change_lane {
                continue;
            }
            let mut candidates: Vec<(f64, f64, usize)> = Vec::new();
            for tgt in &target_pts {
                if tgt.y + (calib.forward_tolerance_m as f64) < src.y {
                    continue;
                }
                let lateral_offset = distance(src.position(), tgt.position());
                if lateral_offset
                    > (calib
                        .max_lane_change_offset_m
                        .max(calib.vehicle_width_m * 1.5)) as f64
                {
                    continue;
                }
                candidates.push(((tgt.y - src.y).abs(), lateral_offset, tgt.index));
            }

            candidates.sort_by(|a, b| match cmp_f64(a.0, b.0) {
                Ordering::Equal => cmp_f64(a.1, b.1),
                other => other,
            });

            let limit = calib
                .max_lane_change_candidates
                .min(calib.cross_lane_neighbors)
                .max(1);

            for (_, offset, target_idx) in candidates.into_iter().take(limit) {
                self.add_edge(
                    NodeKey {
                        lane: source_lane,
                        index: src.index,
                    },
                    NodeKey {
                        lane: target_lane,
                        index: target_idx,
                    },
                    offset,
                    true,
                );
            }
        }
    }

    fn add_edge(&mut self, from: NodeKey, to: NodeKey, distance: f64, lane_change: bool) {
        self.adjacency
            .entry(from)
            .or_insert_with(Vec::new)
            .push(Neighbor {
                node: to,
                distance,
                lane_change,
            });
    }
}

#[derive(Debug)]
struct HeapNode {
    cost: f64,
    estimate: f64,
    key: NodeKey,
    lane_changes: u32,
}

impl HeapNode {
    fn new(cost: f64, estimate: f64, key: NodeKey, lane_changes: u32) -> ReverseHeapNode {
        ReverseHeapNode(HeapNode {
            cost,
            estimate,
            key,
            lane_changes,
        })
    }
}

/// `BinaryHeap`은 최대 힙이므로 `Estimate`가 가장 작은 항목이 먼저 나오도록 래핑한다.
struct ReverseHeapNode(HeapNode);

impl PartialEq for ReverseHeapNode {
    fn eq(&self, other: &Self) -> bool {
        self.0.estimate.eq(&other.0.estimate)
    }
}

impl Eq for ReverseHeapNode {}

impl PartialOrd for ReverseHeapNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ReverseHeapNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .estimate
            .partial_cmp(&self.0.estimate)
            .unwrap_or(Ordering::Equal)
    }
}

impl std::ops::Deref for ReverseHeapNode {
    type Target = HeapNode;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug)]
struct HybridHeapNode {
    cost: f64,
    estimate: f64,
    key: NodeKey,
    heading_idx: u16,
    lane_changes: u32,
}

impl HybridHeapNode {
    fn new(
        cost: f64,
        estimate: f64,
        key: NodeKey,
        heading_idx: u16,
        lane_changes: u32,
    ) -> ReverseHybridHeapNode {
        ReverseHybridHeapNode(HybridHeapNode {
            cost,
            estimate,
            key,
            heading_idx,
            lane_changes,
        })
    }
}

struct ReverseHybridHeapNode(HybridHeapNode);

impl PartialEq for ReverseHybridHeapNode {
    fn eq(&self, other: &Self) -> bool {
        self.0.estimate.eq(&other.0.estimate)
    }
}

impl Eq for ReverseHybridHeapNode {}

impl PartialOrd for ReverseHybridHeapNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ReverseHybridHeapNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .estimate
            .partial_cmp(&self.0.estimate)
            .unwrap_or(Ordering::Equal)
    }
}

impl std::ops::Deref for ReverseHybridHeapNode {
    type Target = HybridHeapNode;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

fn reconstruct_path(
    graph: &PathGraph,
    parent: &HashMap<(NodeKey, u32), (NodeKey, u32)>,
    mut state: (NodeKey, u32),
) -> Vec<PathWaypoint> {
    let mut path = Vec::new();
    while let Some(wp) = graph.waypoint(state.0) {
        path.push(wp.clone());
        if let Some(&prev) = parent.get(&state) {
            state = prev;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

fn reconstruct_path_hybrid(
    graph: &PathGraph,
    parent: &HashMap<(NodeKey, u16, u32), (NodeKey, u16, u32)>,
    mut state: (NodeKey, u16, u32),
) -> Vec<PathWaypoint> {
    let mut path = Vec::new();
    while let Some(wp) = graph.waypoint(state.0) {
        path.push(wp.clone());
        if let Some(&prev) = parent.get(&state) {
            state = prev;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

fn heuristic(graph: &PathGraph, node: NodeKey, goal: NodeKey) -> f64 {
    let node_wp = graph
        .waypoint(node)
        .map(|wp| wp.position())
        .unwrap_or([0.0, 0.0]);
    let goal_wp = graph
        .waypoint(goal)
        .map(|wp| wp.position())
        .unwrap_or([0.0, 0.0]);
    distance(node_wp, goal_wp)
}

fn hybrid_heuristic(
    graph: &PathGraph,
    node: NodeKey,
    heading_idx: u16,
    goal: NodeKey,
    goal_heading_idx: u16,
    heading_bins: u16,
    heading_weight: f64,
) -> f64 {
    let dist = heuristic(graph, node, goal);
    let heading = index_to_heading(heading_idx, heading_bins);
    let goal_heading = index_to_heading(goal_heading_idx, heading_bins);
    let heading_delta = heading_difference(heading, goal_heading).abs();
    dist + heading_delta * heading_weight
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

fn cmp_f64(a: f64, b: f64) -> Ordering {
    a.partial_cmp(&b).unwrap_or(Ordering::Equal)
}

fn heading_between(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[1] - a[1]).atan2(b[0] - a[0])
}

fn heading_difference(a: f64, b: f64) -> f64 {
    normalize_angle(a - b)
}

fn heading_to_index(heading: f64, bins: u16) -> u16 {
    if bins == 0 {
        return 0;
    }
    let step = 2.0 * PI / bins as f64;
    let normalized = normalize_angle_positive(heading);
    let idx = (normalized / step).floor() as i64;
    idx.rem_euclid(bins as i64) as u16
}

fn index_to_heading(idx: u16, bins: u16) -> f64 {
    if bins == 0 {
        return 0.0;
    }
    let step = 2.0 * PI / bins as f64;
    idx as f64 * step
}

fn normalize_angle(mut angle: f64) -> f64 {
    let two_pi = 2.0 * PI;
    while angle <= -PI {
        angle += two_pi;
    }
    while angle > PI {
        angle -= two_pi;
    }
    angle
}

fn normalize_angle_positive(mut angle: f64) -> f64 {
    let two_pi = 2.0 * PI;
    angle = angle % two_pi;
    if angle < 0.0 {
        angle += two_pi;
    }
    angle
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
    #[serde(default = "default_can_change_lane")]
    can_change_lane: bool,
}

fn default_can_change_lane() -> bool {
    true
}

/// 현재 Localization 상태로부터 전역 경로를 재생성하고 DTO로 브로드캐스트한다.
pub fn publish_global_path(
    id: &str,
    graph: &PathGraph,
    calib: &AdasPathGlobalCalibration,
    path_tx: &broadcast::Sender<Arc<DtoAdasGlobalPath>>,
    state: &DtoLocalizationState,
    goal_key: NodeKey,
    blocked: &HashSet<NodeKey>,
    alive_cnt: &mut u32,
    last_log: &mut Instant,
    mode: PathPlanningMode,
) -> Result<PlannedPath, String> {
    if state.map_id != graph.map_id() {
        return Err(format!(
            "Localization map_id({:?})가 경로 지도({:?})와 다릅니다.",
            state.map_id,
            graph.map_id()
        ));
    }

    let position = state.position_map_xy;
    let start_lane = state.lane;
    let heading = localization_heading(state);
    let start_key = graph
        .nearest_waypoint(
            start_lane,
            position,
            heading,
            calib.nearest_search_horizon,
            calib.nearest_heading_cos_threshold,
        )
        .ok_or_else(|| format!("lane={:?}에서 근접 waypoint를 찾지 못했습니다.", start_lane))?;

    let (lane_change_penalty, max_lane_changes) = match mode {
        PathPlanningMode::Normal => (calib.lane_change_penalty_m as f64, calib.max_lane_changes),
        PathPlanningMode::ForceLaneChange => (
            calib.forced_lane_change_penalty_m as f64,
            calib.forced_max_lane_changes,
        ),
    };

    let plan = graph.plan_path(
        calib.global_planner,
        start_key,
        goal_key,
        blocked,
        lane_change_penalty,
        max_lane_changes,
        calib.lane_change_forbidden_lookahead_m as f64,
        calib.lane_change_forbidden_penalty_scale as f64,
    )?;
    if plan.waypoints.is_empty() {
        return Err("경로 결과가 비어 있습니다.".to_string());
    }
    if matches!(mode, PathPlanningMode::ForceLaneChange) && plan.lane_change_count == 0 {
        return Err("차선 변경 강제 모드에서 lane change가 포함되지 않았습니다.".to_string());
    }
    let path_len = plan.waypoints.len();

    let dto_waypoints = plan
        .waypoints
        .iter()
        .map(|wp| {
            DtoPathWaypoint::new(
                wp.lane,
                wp.index as u32,
                [wp.x as f32, wp.y as f32],
                wp.can_change_lane,
            )
        })
        .collect::<Vec<_>>();

    let plan_id = *alive_cnt;
    let dto = Arc::new(DtoAdasGlobalPath::new(
        graph.map_id(),
        dto_waypoints,
        plan_id,
        now_timestamp_ns(),
    ));
    let _ = path_tx.send(dto);
    *alive_cnt = alive_cnt.wrapping_add(1);

    if last_log.elapsed().as_secs_f32() >= 1.0 {
        println!(
            "[{}] 전역 경로 업데이트({:?}): plan_alive={}, start=({:?}, {}), goal=({:?}, {}), waypoint={}개, lane_changes={}",
            id,
            mode,
            plan_id,
            start_key.lane,
            start_key.index,
            goal_key.lane,
            goal_key.index,
            path_len,
            plan.lane_change_count
        );
        *last_log = Instant::now();
    }

    Ok(plan)
}

/// 전역 경로와 Localization 상태를 활용해 로컬(단기) 경로를 구간 절편으로 생성한다.
pub fn try_publish_local_path(
    id: &str,
    calib: &AdasPathLocalCalibration,
    local_tx: &broadcast::Sender<Arc<DtoAdasLocalPath>>,
    global_path: Option<&Arc<DtoAdasGlobalPath>>,
    state: Option<&DtoLocalizationState>,
    alive_cnt: &mut u32,
    last_log: &mut Instant,
) {
    let (global_path, state) = match (global_path, state) {
        (Some(global), Some(state)) => (global, state),
        _ => return,
    };

    if state.map_id != global_path.map_id {
        eprintln!(
            "[{}] 로컬 경로 건너뜀: Localization map({:?}) vs Global map({:?}) 불일치",
            id, state.map_id, global_path.map_id
        );
        return;
    }

    if global_path.waypoints.is_empty() {
        eprintln!(
            "[{}] 전역 경로가 비어 있어 로컬 경로를 생성할 수 없습니다.",
            id
        );
        return;
    }

    let nearest_idx = find_nearest_waypoint_index(global_path, state);
    let window = calib.waypoint_window.max(1);
    let end = (nearest_idx + window).min(global_path.waypoints.len());
    let mut segment = Vec::with_capacity(end - nearest_idx);
    for wp in &global_path.waypoints[nearest_idx..end] {
        segment.push(wp.clone());
    }

    if end == global_path.waypoints.len() {
        if let Some(goal) = global_path.waypoints.last() {
            segment.clear();
            segment.push(goal.clone());
        }
    }

    let plan_id = global_path.alive_cnt;
    let dto = Arc::new(DtoAdasLocalPath::new(
        global_path.map_id,
        plan_id,
        segment,
        *alive_cnt,
        now_timestamp_ns(),
    ));
    if local_tx.send(dto).is_ok() {
        *alive_cnt = alive_cnt.wrapping_add(1);
        if last_log.elapsed().as_secs_f32() >= 1.0 {
            println!(
                "[{}] 로컬 경로 업데이트: plan_alive={} -> segment[{}..{}] ({}개)",
                id,
                plan_id,
                nearest_idx,
                end,
                end - nearest_idx
            );
            *last_log = Instant::now();
        }
    }
}

fn localization_heading(state: &DtoLocalizationState) -> Option<[f64; 2]> {
    state
        .motion_heading_rad
        .map(|ang| [ang.cos(), ang.sin()])
        .or_else(|| {
            let vec = [state.yaw_rad.cos(), state.yaw_rad.sin()];
            if vec[0].is_finite() && vec[1].is_finite() {
                Some(vec)
            } else {
                None
            }
        })
}

/// Localization 위치와 전역 경로에서 가장 가까운 waypoint 인덱스를 찾는다.
pub fn find_nearest_waypoint_index(
    global_path: &DtoAdasGlobalPath,
    state: &DtoLocalizationState,
) -> usize {
    let position = state.position_map_xy;
    let heading =
        localization_heading(state).or_else(|| initial_heading_from_global_path(global_path));

    let mut best_forward: Option<(usize, f64)> = None;
    let mut best_any: Option<(usize, f64)> = None;

    for (idx, wp) in global_path.waypoints.iter().enumerate() {
        let dx = wp.position_xy[0] as f64 - position[0];
        let dy = wp.position_xy[1] as f64 - position[1];
        let dist_sq = dx * dx + dy * dy;

        if let Some(dir) = heading {
            let longitudinal = dx * dir[0] + dy * dir[1];
            let lateral = dx * dir[1] - dy * dir[0];
            if longitudinal >= 0.0 {
                // 프레넷 프레임에서 앞쪽 & 좌우 편차를 모두 반영한 점수.
                let score = longitudinal * longitudinal + lateral * lateral * 2.0;
                if best_forward
                    .map(|(_, best_score)| score < best_score)
                    .unwrap_or(true)
                {
                    best_forward = Some((idx, score));
                }
            }
        }

        if best_any
            .map(|(_, best_score)| dist_sq < best_score)
            .unwrap_or(true)
        {
            best_any = Some((idx, dist_sq));
        }
    }

    best_forward.or(best_any).map(|(idx, _)| idx).unwrap_or(0)
}

/// 현재 시간(ns)을 반환한다. 실패 시 0을 리턴한다.
pub fn now_timestamp_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|dur| dur.as_nanos().min(u64::MAX as u128) as u64)
        .unwrap_or(0)
}

/// 로컬 경로 waypoint를 프레넷 프레임으로 변환한 뒤 5차 다항식으로 스무딩한다.
pub fn smooth_local_path(
    waypoints: &[DtoPathWaypoint],
    state: &DtoLocalizationState,
    sample_count: usize,
) -> Result<Vec<[f32; 2]>, String> {
    if waypoints.len() < 6 {
        return Err("스무딩을 위해 최소 6개 이상의 waypoint가 필요합니다.".to_string());
    }
    if sample_count < 2 {
        return Err("스무딩 샘플 수는 2 이상이어야 합니다.".to_string());
    }

    let origin_xy = state.position_map_xy;
    // 차량 자세는 IMU yaw(roll 보정) 값을 최우선으로 사용하고, 부족할 때만 궤적 기반 추정으로 보완한다.
    let mut heading = Some([state.yaw_rad.cos(), state.yaw_rad.sin()])
        .or_else(|| state.motion_heading_rad.map(|ang| [ang.cos(), ang.sin()]))
        .or_else(|| initial_heading_from_points(waypoints))
        .ok_or("헤딩 각도를 결정할 수 없습니다.")?;

    let norm = (heading[0] * heading[0] + heading[1] * heading[1]).sqrt();
    if norm < 1e-6 {
        heading = [1.0, 0.0];
    } else {
        heading[0] /= norm;
        heading[1] /= norm;
    }

    let tangent = heading;
    let normal = [tangent[1], -tangent[0]];

    let mut s_values = Vec::with_capacity(waypoints.len() + 1);
    let mut d_values = Vec::with_capacity(waypoints.len() + 1);

    let mut cumulative_s = 0.0;
    let mut prev = origin_xy;

    s_values.push(0.0);
    d_values.push(0.0);

    for wp in waypoints {
        let current = [wp.position_xy[0] as f64, wp.position_xy[1] as f64];
        cumulative_s += distance(prev, current);
        prev = current;

        let vec = [current[0] - origin_xy[0], current[1] - origin_xy[1]];
        let d = vec[0] * normal[0] + vec[1] * normal[1];

        s_values.push(cumulative_s);
        d_values.push(d);
    }

    let coeffs = fit_quintic_polynomial(&s_values, &d_values)
        .map_err(|err| format!("5차 다항식 피팅 실패: {}", err))?;

    let max_s = *s_values.last().unwrap_or(&0.0);
    if max_s <= f64::EPSILON {
        // 모든 waypoint가 동일한 위치일 때는 원본 좌표만 반환한다.
        return Ok(waypoints
            .iter()
            .map(|wp| wp.position_xy)
            .collect::<Vec<_>>());
    }

    let mut samples = Vec::with_capacity(sample_count);
    for i in 0..sample_count {
        let t = if sample_count == 1 {
            0.0
        } else {
            i as f64 / (sample_count - 1) as f64
        };
        let s = max_s * t;
        let d = eval_quintic(&coeffs, s);
        let world = [
            origin_xy[0] + s * tangent[0] + d * normal[0],
            origin_xy[1] + s * tangent[1] + d * normal[1],
        ];
        samples.push([world[0] as f32, world[1] as f32]);
    }

    Ok(samples)
}

/// 스무딩된 경로 샘플에서 곡률(2차 미분)을 계산한다.
pub fn curvature_from_smoothed_path(path: &DtoAdasSmoothedPath) -> Result<f64, &'static str> {
    curvature_from_samples(&path.samples_xy)
}

fn initial_heading_from_global_path(path: &DtoAdasGlobalPath) -> Option<[f64; 2]> {
    let mut points = path.waypoints.iter();
    let first = points.next()?;
    for wp in points {
        let dx = wp.position_xy[0] as f64 - first.position_xy[0] as f64;
        let dy = wp.position_xy[1] as f64 - first.position_xy[1] as f64;
        let norm = (dx * dx + dy * dy).sqrt();
        if norm > f64::EPSILON {
            return Some([dx / norm, dy / norm]);
        }
    }
    None
}

fn initial_heading_from_points(points: &[DtoPathWaypoint]) -> Option<[f64; 2]> {
    let mut iter = points.iter();
    let first = iter.next()?;
    for wp in iter {
        let dx = wp.position_xy[0] as f64 - first.position_xy[0] as f64;
        let dy = wp.position_xy[1] as f64 - first.position_xy[1] as f64;
        let norm = (dx * dx + dy * dy).sqrt();
        if norm > f64::EPSILON {
            return Some([dx / norm, dy / norm]);
        }
    }
    None
}

fn curvature_from_samples(samples: &[[f32; 2]]) -> Result<f64, &'static str> {
    if samples.len() < 6 {
        return Err("스무딩 샘플이 부족합니다 (최소 6개 필요)");
    }

    let origin = samples
        .first()
        .map(|pt| [pt[0] as f64, pt[1] as f64])
        .ok_or("스무딩 샘플이 비었습니다")?;

    let mut heading =
        heading_from_samples(samples).ok_or("스무딩 샘플에서 헤딩을 계산할 수 없습니다")?;
    let norm = (heading[0] * heading[0] + heading[1] * heading[1]).sqrt();
    if norm < 1e-6 {
        heading = [1.0, 0.0];
    } else {
        heading[0] /= norm;
        heading[1] /= norm;
    }
    let tangent = heading;
    let normal = [-tangent[1], tangent[0]];

    let mut s_values = Vec::with_capacity(samples.len());
    let mut d_values = Vec::with_capacity(samples.len());
    let mut cumulative_s = 0.0;
    let mut prev = origin;

    for (idx, sample) in samples.iter().enumerate() {
        let current = [sample[0] as f64, sample[1] as f64];
        if idx > 0 {
            cumulative_s += distance(prev, current);
            prev = current;
        }
        let vec = [current[0] - origin[0], current[1] - origin[1]];
        let d = vec[0] * normal[0] + vec[1] * normal[1];
        s_values.push(cumulative_s);
        d_values.push(d);
    }

    let coeffs = fit_quintic_polynomial(&s_values, &d_values)?;
    let curvature = 2.0 * coeffs[2];

    Ok(curvature)
}

fn heading_from_samples(samples: &[[f32; 2]]) -> Option<[f64; 2]> {
    let first = samples.first()?;
    for sample in samples.iter().skip(1) {
        let dx = sample[0] as f64 - first[0] as f64;
        let dy = sample[1] as f64 - first[1] as f64;
        let norm = (dx * dx + dy * dy).sqrt();
        if norm > f64::EPSILON {
            return Some([dx / norm, dy / norm]);
        }
    }
    None
}

fn fit_quintic_polynomial(s_values: &[f64], d_values: &[f64]) -> Result<[f64; 6], &'static str> {
    if s_values.len() != d_values.len() {
        return Err("데이터 길이가 일치하지 않습니다.");
    }
    if s_values.len() < 6 {
        return Err("최소 6개의 샘플이 필요합니다.");
    }

    let scale = s_values.iter().fold(0.0f64, |acc, &s| acc.max(s.abs()));
    if scale <= 1e-12 {
        return Err("s 값 범위가 너무 작습니다.");
    }

    let mut ata = [[0.0f64; 6]; 6];
    let mut atd = [0.0f64; 6];
    for (&s, &d) in s_values.iter().zip(d_values.iter()) {
        let t = s / scale;
        let powers = [1.0, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t];
        for i in 0..6 {
            for j in 0..6 {
                ata[i][j] += powers[i] * powers[j];
            }
            atd[i] += powers[i] * d;
        }
    }

    let coeffs_t = gaussian_solve6(ata, atd).ok_or("선형 시스템을 풀 수 없습니다.")?;

    let mut coeffs = [0.0f64; 6];
    let mut scale_power = 1.0f64;
    for (idx, coef_t) in coeffs_t.iter().enumerate() {
        coeffs[idx] = coef_t / scale_power;
        scale_power *= scale;
    }

    Ok(coeffs)
}

fn gaussian_solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for i in 0..6 {
        // Pivot 선택
        let mut pivot = i;
        for r in i..6 {
            if a[r][i].abs() > a[pivot][i].abs() {
                pivot = r;
            }
        }
        if a[pivot][i].abs() < 1e-9 {
            return None;
        }
        if pivot != i {
            a.swap(i, pivot);
            b.swap(i, pivot);
        }

        let pivot_val = a[i][i];
        for col in i..6 {
            a[i][col] /= pivot_val;
        }
        b[i] /= pivot_val;

        for row in 0..6 {
            if row == i {
                continue;
            }
            let factor = a[row][i];
            for col in i..6 {
                a[row][col] -= factor * a[i][col];
            }
            b[row] -= factor * b[i];
        }
    }

    Some(b)
}

fn eval_quintic(coeffs: &[f64; 6], s: f64) -> f64 {
    let mut result = 0.0;
    let mut power = 1.0;
    for &coef in coeffs {
        result += coef * power;
        power *= s;
    }
    result
}
