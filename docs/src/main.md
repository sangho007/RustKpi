# main.rs - 시스템 오케스트레이터

- 경로: `src/main.rs`
- 계층: 엔트리포인트 / 통합 실행

## 목적
Tokio 멀티스레드 런타임을 초기화하고 RTE 채널, BSW 디바이스 태스크, ASW 인지·제어 태스크를 한 번에 기동합니다. GUI 프리뷰와 센서 로그를 담당하는 `main_runtime::run`으로 제어를 넘겨 전체 파이프라인을 감시합니다.

## 부트스트랩 순서
1. OpenCV OpenCL 가속을 활성화 시도(`core::set_use_opencl(true)`).
2. `rte::rte_main::init()`으로 RTE broadcast 채널 세트를 생성하고 `RteSystem { channels }`를 획득.
3. 카메라 채널을 필요한 만큼 복제해 BSW·ASW 태스크 스폰 시 전달.
4. `tokio::spawn`으로 BSW/ASW 태스크를 비동기 기동.
5. `main_runtime::run(channels)` 호출로 프리뷰 스레드와 신호 구독 루프를 시작.
6. 메인 루프 종료 시 모든 태스크를 중단(`abort`)하고 join한 뒤 종료 코드를 결정.

## 스폰된 태스크
- **BSW**
  - `bsw::ecu_abs_cam::ea_cam_provider`: `CameraCalibration` 설정에 따라 캡처 스레드를 띄우고 RAW DTO를 브로드캐스트합니다.
  - `bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider`: `UltrasonicCalibration` 주기(기본 100ms)로 거리 샘플을 전송합니다.
  - `bsw::ecu_abs_pwm::ea_pca9685_actuator`: `PwmCalibration`을 기반으로 서보 각도와 모터 속도를 PCA9685에 반영합니다.
- **ASW**
  - `asw::vs_lane::runnable_pre_processing`: RAW 프레임을 전처리(`DtoCamProcessed`)해 퍼블리시합니다.
  - `asw::vs_lane::runnable_get_lane_angle`: 버드아이 뷰와 조향각(`DtoCamBirdEyeView`, `DtoCamLaneAngle`)을 계산합니다.
  - `asw::vs_trafficlight::runnable_trafficlight_detection`: HSV + DBSCAN 파이프라인으로 신호등 색(`DtoTrafficLight`)을 판정합니다.
  - `asw::forwardcollision_ultrasonic::runnable_obstacle_detection`: 거리 임계값(`ForwardCollisionCalibration`)으로 장애물 이벤트(`DtoUltraSonicObstacle`)를 생성합니다.

## 런타임/종료 제어
- `main_runtime::run`이 GUI 프리뷰 스레드(`util::preview_runtime`)를 조건부로 띄우고, 최신 프레임과 센서 데이터를 구독해 화면 출력 및 로그를 수행합니다.
- Ctrl-C 또는 GUI quit 이벤트를 감지하면 루프를 빠져나가고, 스폰된 모든 태스크를 정리한 뒤 OpenCV `Result`에 따라 프로세스 종료 코드를 선택합니다.

## 동시성 및 장애 대응
- 태스크는 `tokio::spawn`으로 실행되며, 필요한 경우 내부에서 `std::thread::Builder` 또는 `spawn_blocking`으로 블로킹 구간을 격리합니다.
- 채널 `Lagged` 또는 `Closed` 이벤트는 각 태스크에서 경고 로그를 남기고, 상황에 따라 루프를 유지하거나 종료합니다.

## TODO
- `asw::adas` 융합 태스크를 실제 제어 경로로 연결합니다.
- GUI/텔레메트리 옵션 정리와 성능 계측 루틴을 보강합니다.
