# adas_cod.rs — ADAS 제어 캘리브레이션

- 경로: `src/calibration/adas_cod.rs`
- 계층: Calibration / ADAS Control

## 목적
- 횡방향·종방향 제어 루프가 사용할 게인과 임계값을 코드 변화 없이 튜닝할 수 있도록 중앙에서 정의합니다.
- 조향/속도 명령의 응답성과 안전 한계를 모두 캘리브레이션 레벨에서 관리합니다.

## 주요 구조체
- `AdasLateralCalibration`: 차량 yaw를 기준으로 한 직선 참조 대비 스무딩 궤적 횡오차를 PID 제어하기 위한 게인, 각도 범위, 레이트 리밋, 샘플 인덱스, 채널 인덱스를 제공합니다.
- `AdasLongitudinalCalibration`: 초음파·신호등·경로 상태로 속도를 결정하는 제어 루프 주기, 순항/감속 속도, 감속·정지 거리, 곡률 감속 게인, 속도 레이트 리밋 등을 정의합니다.

## 기본값 특징
- 측면 제어는 중립 90도, ±90도 범위를 기준으로 PID 제어를 수행하며 한 루프당 최대 10도까지 각도 변화를 허용합니다. 차량 현재 yaw를 따라 형성한 직선 참조와 스무딩 궤적의 지정 샘플(기본 8번째) 간 횡오차를 제어 대상 값으로 사용합니다.
- 종방향 제어는 50ms 주기를 기본으로 하고, 60% 순항 속도 / 25% 감속 속도 / 35cm 정지 거리를 사용합니다. 황색·소등 신호나 장애물 근접 시 감속(0.6배), 차선 변경 요청 시 감속이 적용됩니다.

## 주요 파라미터
| 필드 | 설명 |
| --- | --- |
| `pid_kp`, `pid_ki`, `pid_kd` | 횡오차 PID 제어 게인 |
| `pid_integral_limit` | 적분 항 누적 한계 (m·s) |
| `pid_sample_index` | 횡오차 계산에 사용할 스무딩 샘플 인덱스 |
| `max_servo_delta_deg` | 루프당 허용되는 서보 각 변화량 |
| `curvature_slowdown_threshold` | 곡률이 해당 값 이상이면 종방향 gain 0.8배 적용 |
| `stop_request_hold_time` / `stop_release_hold_time` | 정지 요청 유지 시간 / 해제 대기 시간 |
| `max_accel_delta_percent` | 루프당 허용되는 가속 증가폭(%) |
| `max_decel_delta_percent` | 루프당 허용되는 감속 감소폭(%) |

## 운용 팁
- **경로 가드:** 스무딩 경로(`DtoAdasSmoothedPath`)가 아직 준비되지 않은 경우 `runnable_adas_longitudinal`은 `no_path` 이유로 강제 정지 상태를 유지합니다. 경로 모듈이 안정적으로 동작하는지 먼저 확인하세요.
- **속도 레이트 리밋:** `max_accel_delta_percent`와 `max_decel_delta_percent`를 조정하여 RC카의 주행 중 튀는 가속·감속을 완화할 수 있습니다. 값이 너무 작으면 응답이 둔해지고, 너무 크면 급격한 변동이 발생합니다.
- **정지 판단:** 초음파·신호등·경로 상태 중 하나라도 정지 조건을 만족하면 `stop_request_hold_time` 이상 지속될 때 완전 정지합니다. 이후 `stop_release_hold_time` 동안은 정지 상태를 유지한 뒤에만 가속이 허용됩니다.

튜닝 시에는 동일 파라미터 세트를 `src/calibration/adas_cod.rs`에서 수정한 뒤, 센서 데이터(초음파·신호등)와 경로 시각화를 함께 모니터링하며 변경 전후 동작을 비교하는 것이 좋습니다.
