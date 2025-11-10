#!/usr/bin/env python3
import json
from collections import deque, defaultdict
from math import hypot


def load_map(path):
    with open(path, 'r') as f:
        data = json.load(f)
    def norm(lst):
        out = []
        for i, wp in enumerate(lst):
            pos = wp["position"]
            can = wp.get("can_change_lane", True)
            out.append({"index": i, "x": float(pos[0]), "y": float(pos[1]), "can": bool(can)})
        return out
    return norm(data["inner_waypoint"]), norm(data["outer_waypoint"])


def build_same_lane_edges(points, max_same_lane_distance_m, forward_tolerance_m, same_lane_neighbors, use_y_forward):
    adj = defaultdict(list)
    for cur in points:
        cidx = cur["index"]
        cx, cy = cur["x"], cur["y"]
        candidates = []
        for oth in points:
            if oth["index"] == cidx:
                continue
            if use_y_forward:
                if (oth["y"] + forward_tolerance_m) < cy:
                    # 뒤쪽 후보는 스킵
                    continue
            dist = hypot(oth["x"] - cx, oth["y"] - cy)
            if dist > max_same_lane_distance_m:
                continue
            candidates.append((dist, oth["index"]))
        candidates.sort(key=lambda t: t[0])
        for dist, tidx in candidates[:same_lane_neighbors]:
            adj[cidx].append((tidx, dist, False))
    return adj


def build_cross_lane_edges(src_lane, tgt_lane, max_lane_change_offset_m, vehicle_width_m, forward_tolerance_m, cross_lane_neighbors, max_lane_change_candidates, use_y_forward):
    adj = defaultdict(list)
    for src in src_lane:
        if not src["can"]:
            continue
        cx, cy = src["x"], src["y"]
        cand = []
        for tgt in tgt_lane:
            if use_y_forward:
                if (tgt["y"] + forward_tolerance_m) < cy:
                    continue
            lateral = hypot(tgt["x"] - cx, tgt["y"] - cy)
            limit = max(max_lane_change_offset_m, vehicle_width_m * 1.5)
            if lateral > limit:
                continue
            cand.append((abs(tgt["y"] - cy), lateral, tgt["index"]))
        cand.sort(key=lambda t: (t[0], t[1]))
        limit_n = max(1, min(max_lane_change_candidates, cross_lane_neighbors))
        for _, lateral, tidx in cand[:limit_n]:
            adj[src["index"]].append((tidx, lateral, True))
    return adj


def bfs_connected(adj, start_idx, goal_idx):
    q = deque([start_idx])
    seen = {start_idx}
    parent = {start_idx: None}
    while q:
        u = q.popleft()
        if u == goal_idx:
            # reconstruct
            path = []
            cur = u
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return list(reversed(path))
        for v, _, _ in adj.get(u, []):
            if v not in seen:
                seen.add(v)
                parent[v] = u
                q.append(v)
    return None


def main():
    inner, outer = load_map("src/asw/lib/map_data_4lane_quantized_chagable.json")

    # Calibration snapshot (must match Rust defaults)
    vehicle_width_m = 0.15
    max_same_lane_distance_m = 0.2
    forward_tolerance_m = 0.02
    same_lane_neighbors = 3
    cross_lane_neighbors = 4
    max_lane_change_offset_m = 1.0
    max_lane_change_candidates = 8

    # Scenario: Crossroad, start: Outer index=79, goal: Outer index=50
    start_idx = 79
    goal_idx = 50

    for use_y_forward in (True, False):
        same = build_same_lane_edges(
            outer,
            max_same_lane_distance_m,
            forward_tolerance_m,
            same_lane_neighbors,
            use_y_forward,
        )
        cross1 = build_cross_lane_edges(
            outer,
            inner,
            max_lane_change_offset_m,
            vehicle_width_m,
            forward_tolerance_m,
            cross_lane_neighbors,
            max_lane_change_candidates,
            use_y_forward,
        )
        cross2 = build_cross_lane_edges(
            inner,
            outer,
            max_lane_change_offset_m,
            vehicle_width_m,
            forward_tolerance_m,
            cross_lane_neighbors,
            max_lane_change_candidates,
            use_y_forward,
        )

        # merge to a flat adjacency on lane=outer only to test same-lane travel
        adj = defaultdict(list)
        for u, lst in same.items():
            adj[u].extend(lst)
        # Optional: if you want to allow lane change, also wire cross edges (inner index space clashes omitted here)

        path = bfs_connected(adj, start_idx, goal_idx)
        label = "WITH y-forward filter" if use_y_forward else "WITHOUT y-forward filter"
        if path:
            print(f"{label}: reachable in {len(path)-1} hops. path(head/tail)={path[:5]}...{path[-5:]}")
        else:
            print(f"{label}: NOT reachable on same lane.")


if __name__ == "__main__":
    main()

