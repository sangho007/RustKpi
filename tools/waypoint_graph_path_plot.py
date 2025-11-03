#!/usr/bin/env python3
"""차선 변경 가능 여부를 반영해 waypoint 경로를 생성하고 시각화하는 유틸리티."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import matplotlib


def setup_matplotlib(show: bool) -> Any:
    if not show:
        matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt_module

    return plt_module

VEHICLE_WIDTH = 0.18  # m
VEHICLE_LENGTH = 0.25  # m
MAX_LANE_CHANGE_OFFSET = 1.0  # m, 차선 간 최소 거리 여유
Y_FORWARD_TOL = 0.02  # y축 기준 허용 역방향 여유 (m)
LANE_CHANGE_PENALTY = 1.0  # m, 차선 변경에 대한 추가 비용
MAX_LANE_CHANGES = 1  # 허용되는 최대 차선 변경 횟수
MAX_LANE_CHANGE_CANDIDATES = 8  # 차선 변경 시 고려할 후보 수
SAME_LANE_NEIGHBORS = 4  # 같은 차선에서 고려할 최근접 이웃 수
CROSS_LANE_NEIGHBORS = 3  # 차선 변경 시 최대 연결 수
MAX_SAME_LANE_DISTANCE = 0.1  # 같은 차선 연결에 허용되는 최대 거리 (m)


@dataclass(frozen=True)
class Waypoint:
    lane: str
    index: int
    x: float
    y: float
    can_change_lane: bool

    @property
    def key(self) -> Tuple[str, int]:
        return (self.lane, self.index)

    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "lane,index 형태로 지정한 출발지와 목적지 사이에, "
            "차선 변경 가능 여부를 지키며 waypoint 기반 경로를 탐색하고 시각화합니다."
        )
    )
    parser.add_argument("map_file", help="맵 JSON 파일 경로")
    parser.add_argument("start", help="출발지 lane,index (예: inner,0)")
    parser.add_argument("goal", help="도착지 lane,index (예: outer,1200)")
    parser.add_argument(
        "--save",
        type=Path,
        help="결과 플롯을 저장할 경로 (미지정 시 기본값 figure.png)",
    )
    parser.add_argument(
        "--figure",
        default="figure.png",
        help="--save 미사용 시 기본 저장 파일명 (기본값: figure.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="플롯을 화면에 표시합니다. --save 미사용 시 파일로 저장하지 않습니다.",
    )
    return parser.parse_args()


def parse_lane_index(spec: str) -> Tuple[str, int]:
    for sep in (",", ":", "/"):
        if sep in spec:
            lane, idx = spec.split(sep, 1)
            break
    else:
        raise ValueError(f"lane,index 형식을 인식할 수 없습니다: {spec}")
    lane = lane.strip().lower()
    if lane not in {"inner", "outer", "inner_waypoint", "outer_waypoint"}:
        raise ValueError(f"lane 값은 inner/outer 만 허용됩니다: {lane}")
    return lane, int(idx.strip())


def lane_key(lane: str) -> str:
    if lane.startswith("inner"):
        return "inner_waypoint"
    if lane.startswith("outer"):
        return "outer_waypoint"
    return lane


def normalize_waypoint(item: Any) -> Tuple[float, float, bool]:
    if isinstance(item, dict):
        pos = item.get("position")
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            x, y = float(pos[0]), float(pos[1])
        else:
            raise ValueError("position 필드가 올바르지 않습니다.")
        can_change = bool(item.get("can_change_lane", True))
        return x, y, can_change
    if isinstance(item, (list, tuple)) and len(item) >= 2:
        return float(item[0]), float(item[1]), True
    raise ValueError("waypoint 항목을 파싱할 수 없습니다.")


def load_waypoints(data: Dict[str, Any], lane: str) -> List[Waypoint]:
    key = lane_key(lane)
    raw_list = data.get(key)
    if not isinstance(raw_list, Iterable):
        return []

    waypoints: List[Waypoint] = []
    for idx, item in enumerate(raw_list):
        try:
            x, y, can_change = normalize_waypoint(item)
        except ValueError:
            continue
        waypoints.append(
            Waypoint(
                lane="inner" if key == "inner_waypoint" else "outer",
                index=idx,
                x=x,
                y=y,
                can_change_lane=can_change,
            )
        )
    return waypoints


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def edge_cost(src: Waypoint, dst: Waypoint) -> float:
    base = distance(src.position(), dst.position())
    if src.lane != dst.lane:
        return base + LANE_CHANGE_PENALTY
    return base


def build_graph(
    inner_pts: List[Waypoint],
    outer_pts: List[Waypoint],
) -> Dict[Tuple[str, int], List[Tuple[Tuple[str, int], float]]]:
    graph: Dict[Tuple[str, int], List[Tuple[Tuple[str, int], float]]] = {}

    def add_edge(src: Waypoint, dst: Waypoint) -> None:
        graph.setdefault(src.key, []).append((dst.key, edge_cost(src, dst)))

    # 같은 차선 내 연결 (가까운 순으로 전방 이웃 선택)
    for lane_pts in (inner_pts, outer_pts):
        for current in lane_pts:
            candidates: List[Tuple[float, Waypoint]] = []
            for other in lane_pts:
                if other is current:
                    continue
                if other.y + Y_FORWARD_TOL < current.y:
                    continue
                dist = distance(current.position(), other.position())
                if dist > MAX_SAME_LANE_DISTANCE:
                    continue
                candidates.append((dist, other))
            candidates.sort(key=lambda item: item[0])
            for dist, other in candidates[:SAME_LANE_NEIGHBORS]:
                add_edge(current, other)

    # 차선 변경 연결
    if inner_pts and outer_pts:
        def connect_lanes(
            source_lane: List[Waypoint],
            target_lane: List[Waypoint],
        ) -> None:
            if not source_lane or not target_lane:
                return

            for i, src_wp in enumerate(source_lane):
                if not src_wp.can_change_lane:
                    continue

                candidates: List[Tuple[float, float, Waypoint]] = []
                for tgt_wp in target_lane:
                    if not tgt_wp.can_change_lane:
                        continue
                    if tgt_wp.y + Y_FORWARD_TOL < src_wp.y:
                        continue

                    lateral_offset = distance(src_wp.position(), tgt_wp.position())
                    if lateral_offset > max(MAX_LANE_CHANGE_OFFSET, VEHICLE_WIDTH * 1.5):
                        continue

                    candidates.append((abs(tgt_wp.y - src_wp.y), lateral_offset, tgt_wp))

                candidates.sort(key=lambda item: (item[0], item[1]))
                for _, _, tgt_wp in candidates[:CROSS_LANE_NEIGHBORS]:
                    add_edge(src_wp, tgt_wp)

        connect_lanes(inner_pts, outer_pts)
        connect_lanes(outer_pts, inner_pts)

    return graph


def a_star(
    graph: Dict[Tuple[str, int], List[Tuple[Tuple[str, int], float]]],
    nodes: Dict[Tuple[str, int], Waypoint],
    start: Tuple[str, int],
    goal: Tuple[str, int],
) -> List[Waypoint]:
    if start not in nodes:
        raise KeyError(f"출발 노드가 존재하지 않습니다: {start}")
    if goal not in nodes:
        raise KeyError(f"도착 노드가 존재하지 않습니다: {goal}")

    def heuristic(node_key: Tuple[str, int]) -> float:
        return distance(nodes[node_key].position(), nodes[goal].position())

    open_heap: List[Tuple[float, float, Tuple[str, int], int]] = []
    heapq.heappush(open_heap, (heuristic(start), 0.0, start, 0))

    g_score: Dict[Tuple[Tuple[str, int], int], float] = {(start, 0): 0.0}
    parent: Dict[Tuple[Tuple[str, int], int], Tuple[Tuple[str, int], int]] = {}

    while open_heap:
        _, current_cost, current_key, current_changes = heapq.heappop(open_heap)
        current_state = (current_key, current_changes)

        if current_key == goal:
            return reconstruct_path(parent, nodes, current_state)

        for neighbor_key, step_cost in graph.get(current_key, []):
            extra_change = 1 if nodes[neighbor_key].lane != nodes[current_key].lane else 0
            next_changes = current_changes + extra_change
            if next_changes > MAX_LANE_CHANGES:
                continue

            tentative = current_cost + step_cost
            neighbor_state = (neighbor_key, next_changes)
            if tentative < g_score.get(neighbor_state, float("inf")):
                g_score[neighbor_state] = tentative
                parent[neighbor_state] = current_state
                heapq.heappush(
                    open_heap,
                    (tentative + heuristic(neighbor_key), tentative, neighbor_key, next_changes),
                )

    raise RuntimeError("경로를 찾을 수 없습니다.")


def reconstruct_path(
    parent: Dict[Tuple[Tuple[str, int], int], Tuple[Tuple[str, int], int]],
    nodes: Dict[Tuple[str, int], Waypoint],
    goal_state: Tuple[Tuple[str, int], int],
) -> List[Waypoint]:
    path: List[Waypoint] = []
    state = goal_state
    while True:
        node_key, _ = state
        path.append(nodes[node_key])
        if state not in parent:
            break
        state = parent[state]
    path.reverse()
    return path


def plot_environment(
    inner_pts: List[Waypoint],
    outer_pts: List[Waypoint],
    path: List[Waypoint],
    plt_module: Any,
    output: Optional[Path],
    show: bool,
) -> None:
    fig, ax = plt_module.subplots(figsize=(8, 8))

    if inner_pts:
        ax.scatter(
            [wp.x for wp in inner_pts],
            [wp.y for wp in inner_pts],
            s=8,
            color="#1f77b4",
            alpha=0.5,
            label="inner lane",
        )
    if outer_pts:
        ax.scatter(
            [wp.x for wp in outer_pts],
            [wp.y for wp in outer_pts],
            s=8,
            color="#ff7f0e",
            alpha=0.5,
            label="outer lane",
        )

    if path:
        ax.plot(
            [wp.x for wp in path],
            [wp.y for wp in path],
            color="#2ca02c",
            linewidth=2.0,
            label="planned path",
        )
        ax.scatter([path[0].x], [path[0].y], color="green", s=60, marker="o", label="start")
        ax.scatter([path[-1].x], [path[-1].y], color="red", s=60, marker="x", label="goal")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.legend()
    if output is not None:
        fig.savefig(output, bbox_inches="tight")
    if show:
        plt_module.show()
    plt_module.close(fig)


def main() -> None:
    args = parse_args()
    plt_module = setup_matplotlib(args.show)
    map_path = Path(args.map_file)
    if not map_path.exists():
        raise FileNotFoundError(f"맵 파일을 찾을 수 없습니다: {map_path}")

    with map_path.open("r", encoding="utf-8") as fp:
        data = json.load(fp)

    inner_pts = load_waypoints(data, "inner")
    outer_pts = load_waypoints(data, "outer")

    nodes: Dict[Tuple[str, int], Waypoint] = {
        wp.key: wp for lane_pts in (inner_pts, outer_pts) for wp in lane_pts
    }

    graph = build_graph(inner_pts, outer_pts)

    start_lane, start_idx = parse_lane_index(args.start)
    goal_lane, goal_idx = parse_lane_index(args.goal)
    start_key = (lane_key(start_lane).replace("_waypoint", ""), start_idx)
    goal_key = (lane_key(goal_lane).replace("_waypoint", ""), goal_idx)

    path = a_star(graph, nodes, start_key, goal_key)

    if args.save:
        save_path: Optional[Path] = args.save
    elif args.show:
        save_path = None
    else:
        save_path = Path(args.figure)
    plot_environment(inner_pts, outer_pts, path, plt_module, save_path, args.show)

    print("경로 waypoint (lane,index):")
    for wp in path:
        print(f"{wp.lane},{wp.index} -> ({wp.x:.5f}, {wp.y:.5f})")
    if save_path is not None:
        print(f"총 {len(path)}개 waypoint, 결과 플롯 저장: {save_path}")
    if args.show:
        print("플롯이 화면에 표시되었습니다.")


if __name__ == "__main__":
    main()
