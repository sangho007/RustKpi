# adas_path.rs — 경로 계획 캘리브레이션

- 경로: `src/calibration/adas_path.rs`
- 계층: Calibration / ADAS Path

## 목적
- 경로 계획 러너블이 사용할 전역·로컬 파라미터를 코드 변경 없이 조정해 다양한 주행 환경에 대응합니다.
- 차선 변경 비용, 장애물 차단, 로컬 스무딩 샘플 수 등 핵심 값을 중앙에서 관리합니다.

## 주요 구조체
- `AdasPathGlobalCalibration`: 전역 경로 탐색(A* 기반)에서 사용하는 물리 파라미터, 후보 탐색 범위, 차선 변경 제약, 장애물 회피 시간을 정의합니다.
- `AdasPathLocalCalibration`: 전역 경로에서 잘라낸 로컬 구간을 스무딩하고 샘플링할 때 필요한 파라미터를 제공합니다.

## 전역 경로 파라미터 (요약)
| 필드 | 설명 | 기본값 |
| --- | --- | --- |
| `replanning_period` | 재탐색 주기 | 2초 |
| `vehicle_width_m` / `vehicle_length_m` | 차량 궤적 계획 시 사용하는 차폭/길이 | 0.18m / 0.25m |
| `max_lane_change_offset_m` | 허용할 최대 횡방향 오프셋 | 1.0m |
| `forward_tolerance_m` | 동일 차선에서 뒤로 물러나도 인정하는 여유 거리 | 0.02m |
| `lane_change_penalty_m` | 차선 변경 비용 (값이 클수록 차선 유지 선호) | 1.0m |
| `forced_lane_change_penalty_m` | 강제 차선 변경 모드 시 비용 | -0.5m |
| `max_lane_changes` / `forced_max_lane_changes` | 기본/강제 모드에서 허용할 최대 차선 변경 횟수 | 1 / 2 |
| `same_lane_neighbors` / `cross_lane_neighbors` | 동일/다른 차선에서 탐색할 후보 노드 수 | 4 / 3 |
| `max_same_lane_distance_m` | 동일 차선에서 연결 허용 거리 | 0.1m |
| `max_lane_change_candidates` | 차선 변경 후보 수 상한 | 8 |
| `nearest_search_horizon` | 시작 지점 후보로 살펴볼 waypoint 수 | 12 |
| `lane_change_retry_cooldown` | 강제 차선 변경 재시도 최소 간격 | 500ms |
| `obstacle_block_margin_m` | 장애물 앞 waypoint 차단 여유 거리 | 0.2m |
| `obstacle_block_timeout` | 차단 유지 시간 | 2초 |
| `obstacle_block_heading_tolerance_deg` | 장애물 차단 시 허용할 헤딩 편차(도) | 10° |

## 로컬 경로 파라미터
| 필드 | 설명 | 기본값 |
| --- | --- | --- |
| `waypoint_window` | 전역 경로에서 잘라낼 waypoint 수 | 7 |
| `smoothing_sample_count` | 스무딩 후 생성할 샘플 개수 | 20 |

## 튜닝 가이드
- **차선 변경 민감도**: `lane_change_penalty_m`를 키우면 동일 차선을 유지하고, 작게 하면 차선 변경을 더 쉽게 허용합니다. 강제 차선 변경 시에는 `forced_lane_change_penalty_m`가 별도로 적용되므로 둘의 균형을 맞추세요.
- **장애물 회피**: 초음파 장애물이 감지되면 전역 경로 러너블이 `obstacle_block_margin_m`만큼 앞쪽 waypoint를 차단합니다. 거리 센서 정확도에 따라 여유 폭과 `obstacle_block_timeout`을 조정하세요.
- **로컬 경로 품질**: 스무딩 샘플 수를 늘리면 곡선이 부드럽지만 연산량이 늘고, 너무 작으면 곡선이 꺾일 수 있습니다. 차량 회전 반경에 맞춰 `waypoint_window`와 함께 테스트하세요.

튜닝 변경 시 `cargo fmt` 후 `cargo check` 혹은 주행 테스트와 함께 `Path View`(SDL 프리뷰 창의 `M` 키)에서 전역/로컬 경로와 차량 위치가 의도대로 변하는지 확인하는 것을 권장합니다.
