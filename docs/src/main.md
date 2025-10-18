# main.rs — 시스템 오케스트레이터

- 경로: `src/main.rs`
- 계층: 엔트리포인트 / 통합 실행

## 목적
Tokio 멀티스레드 런타임에서 RTE 채널을 초기화하고, BSW(센서/액추에터)와 ASW(인지/제어) 태스크를 생성·오케스트레이션합니다. 디버깅 시 카메라 처리 결과를 GUI로 표시합니다.

## 역할 & 데이터 흐름
- RTE 초기화: `rte::rte_main::init()`로 모든 Broadcast 채널 생성.
- BSW 태스크:
  - `bsw::ecu_abs_cam::ea_cam_provider` (카메라 Raw 프레임 송신)
  - `bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider` (초음파 Raw 송신)
  - `bsw::ecu_abs_pca9685::ea_pca9685_actuator` (서보/DC 모터 명령 수신)
- ASW 태스크:
  - `asw::vs_lane::runnable_pre_processing` (Preprocess → Processed)
  - `asw::vs_lane::runnable_get_lane_angle` (Bird’s‑eye, LaneAngle 산출)
  - `asw::uss_forwardcollision::runnable_obstacle_detection` (장애물 이벤트)
  - `asw::vs_trafficlight::runnable_trafficlight_detection` (신호등 상태)
- 디버그(옵션): `processed`, `bird_eye`, `lane_angle` 구독 후 GUI 출력.

## 동시성
- `tokio::spawn` + `spawn_blocking` 조합으로 CPU/GPU 바운드 처리와 GUI 블로킹 호출을 격리.
- Broadcast 채널로 fan‑out, 각 소비자는 `subscribe()`로 독립 버퍼 확보.

## 에러 처리
- 태스크 Join 에러를 OpenCV `Error`로 매핑하여 상위로 전달.
- 채널 `Lagged/Closed`는 로그 후 루프 유지 또는 종료.

## 구성/상수
- 디버그 GUI 스위치: `DEBUG_ON` (기본 false)

## TODO
- `asw/adas` 통합 제어 융합 태스크 연결
- GUI/텔레메트리 옵션 정리 및 성능 계측 고도화

