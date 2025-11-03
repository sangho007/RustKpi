# adas_path_lib.rs — 경로 계획 라이브러리

- 경로: `src/asw/lib/adas_path_lib.rs`
- 계층: ASW / Library (Path Planning)

## 목적
지도 JSON을 로드해 inner/outer 레인 waypoint 그래프를 구성하고, A* 기반 전역 경로 탐색과 로컬 궤적 스무딩 유틸리티를 제공합니다. 전역 경로/로컬 경로 러너블이 공통 로직을 재사용할 수 있도록 분리되어 있습니다.

## 주요 구조체
- `PathGraph`: 지도(`LocalizationMapId`)의 inner/outer waypoint와 인접 리스트를 보관합니다. `load()` 시 지도 JSON을 파싱하고 `build_edges()`로 차선 내/간 연결을 생성합니다.
- `NodeKey { lane, index }`: waypoint 식별자. `LocalizationLane`와 인덱스를 조합합니다.
- `PlannedPath { waypoints, lane_change_count }`: A* 탐색 결과.
- `RawMap`, `RawWaypoint`: 지도 JSON 역직렬화용 구조체.

## 전역 경로 탐색
- `PathGraph::nearest_waypoint(lane, position, horizon)`: 현재 위치와 가장 가까운 waypoint 후보를 찾습니다.
- `PathGraph::plan_path(start, goal, blocked, lane_change_penalty, max_lane_changes)`: 차선 변경 비용과 차단 노드를 고려한 A* 탐색을 수행합니다.
- `publish_global_path(...)`: `PlannedPath`를 `DtoAdasGlobalPath`로 변환해 브로드캐스트하고, 1초 주기로 로그를 출력합니다.
- `compute_blocked_nodes(...)`: 장애물 거리(threshold)만큼 경로 waypoint를 차단 리스트에 추가합니다.
- `refresh_blocked_nodes(...)`: 만료된 차단 항목을 제거하고 캐시를 갱신합니다.

## 로컬 경로 및 스무딩
- `try_publish_local_path(...)`: 전역 경로와 로컬라이제이션 상태에서 차량 주변 일정 구간(`AdasPathLocalCalibration::waypoint_window`)을 잘라 `DtoAdasLocalPath`로 게시합니다.
- `smooth_local_path(waypoints, state, sample_count)`: 프레네 좌표로 변환 후 5차 다항식 곡선을 피팅해 균일 간격 샘플을 생성합니다. 실패 시 원본 waypoint를 반환합니다.
- `curvature_from_smoothed_path(...)`: 스무딩 샘플에서 곡률을 계산해 횡방향 제어에 활용합니다.
- `determine_lane_change_state(...)`: 로컬 경로에 inner/outer waypoint가 혼재하는지 확인해 `AdasLaneChangeState`를 판정합니다.

## 헬퍼 함수
- `distance`, `cmp_f64`, `heuristic`: A* 탐색의 거리 계산 및 우선순위 큐 비교에 사용.
- `fit_quintic_polynomial`, `eval_quintic`: 5차 다항식 회귀 및 평가.
- `now_timestamp_ns()`: `SystemTime::now()` 기반 1ns 타임스탬프 생성.

## 연관 DTO
- `DtoAdasGlobalPath`, `DtoAdasLocalPath`, `DtoAdasSmoothedPath`, `DtoPathWaypoint`
- `DtoLocalizationState`: 현재 위치/헤딩을 제공해 로컬 경로 절단·스무딩에 사용됩니다.

## 튜닝 지점
- `calibration::adas_path::AdasPathGlobalCalibration`: 차선 변경 비용, 후보 개수, 장애물 차단 거리, 재탐색 주기.
- `calibration::adas_path::AdasPathLocalCalibration`: 로컬 절편 window 크기와 스무딩 샘플 수.
- 지도 JSON(`LocalizationMapId::json_asset`)에서 `can_change_lane` 플래그를 설정하면 차선 변경 가능 구간을 제어할 수 있습니다.
