# RustKpi

자율주행 RC 카 플랫폼에서 Classic AUTOSAR 계층(ASW/RTE/BSW)을 간소화해 모사하고, Rust 기반으로 ADAS 핵심 기능을 실험하는 프로젝트입니다. 차선 유지, 신호등 대응, 전방 장애물 대응을 우선 구현하고, 추후 C/C++ 포팅과 성능·안전성 비교를 목표로 합니다.

## 프로젝트 목표
- Classic AUTOSAR 계층 구조(BSW ↔ RTE ↔ ASW)와 실행 흐름을 Rust로 재현
- 동일 기능을 Rust와 C/C++로 각각 구현하여 안전성·성능·개발 경험 비교
- 라즈베리파이 + RC 카 + 센서(카메라, 초음파, PCA9685 기반 모터/서보)로 실제 주행 시나리오 검증

## 모사 시나리오
1. **LFA + LCA (차선 유지 & 변경)**  
   - 차선을 추종하며 주행, 전방 차량 감지 시 안전한 차선 변경
2. **TLA (신호등 대응)**  
   - 신호등 색상을 인식하여 정지 후 출발, 보행자 신호 등 확장 고려
3. **FCA (전방 충돌 회피)**  
   - 돌발 보행자/장애물 감지 시 긴급 정차 및 경고

## 시스템 아키텍처
```
┌──────────────────────────────────────────────────────────────────────┐
│   Application Software (ASW)                                         │ vs_lane, vs_trafficlight, forwardcollision_ultrasonic, adas_cod
├──────────────────────────────────────────────────────────────────────┤
│   Runtime Environment (RTE)                                          │ DTO, Broadcast Channels, Scheduling
├──────────────────────────────────────────────────────────────────────┤
│   Basic Software (BSW)                                               │ Camera, Ultrasonic, PCA9685, USB/TCP Gateway, IMU Decode
└──────────────────────────────────────────────────────────────────────┘
```

| 계층 | 주요 모듈 | 경로 |
| --- | --- | --- |
| ASW | `vs_lane`, `vs_trafficlight`, `forwardcollision_ultrasonic`, `adas_cod`, `adas_localization`, `adas_path_local`, `adas_path_global` | `src/asw/` |
| RTE | `rte_main`, `rte_dto`, `lib` | `src/rte/` |
| BSW | `ecu_abs_cam`, `ecu_abs_ultrasonic`, `ecu_abs_pwm`, `ecu_abs_com`, `ecu_abs_imu`, `lib` | `src/bsw/` |
| Calibration | `lane`, `traffic_light`, `pwm`, `ultrasonic`, `forward_collision`, `com`, `adas_cod` | `src/calibration/` |

```
src
├── asw
│   ├── lib
│   │   ├── forwardcollision_ultrasonic_lib.rs
│   │   ├── vs_lane_lib.rs
│   │   └── vs_trafficlight_lib.rs
│   ├── adas_cod.rs
│   ├── adas_localization.rs
│   ├── adas_path_global.rs
│   ├── adas_path_local.rs
│   ├── forwardcollision_ultrasonic.rs
│   ├── vs_lane.rs
│   └── vs_trafficlight.rs
├── bsw
│   ├── lib
│   │   ├── cam_lib.rs
│   │   ├── imu_proto.rs
│   │   ├── pwm_lib.rs
│   │   └── ultrasonic_lib.rs
│   ├── ecu_abs_cam.rs
│   ├── ecu_abs_com.rs
│   ├── ecu_abs_imu.rs
│   ├── ecu_abs_pwm.rs
│   └── ecu_abs_ultrasonic.rs
├── calibration
│   ├── adas_cod.rs
│   ├── com.rs
│   ├── forward_collision.rs
│   ├── lane
│   ├── pwm.rs
│   ├── traffic_light.rs
│   └── ultrasonic.rs
├── rte
│   ├── rte_dto.rs
│   ├── rte_main.rs
│   └── lib
├── util
│   ├── mod.rs
│   ├── preview_runtime.rs
│   ├── preview_window.rs
│   └── sdl_env.rs
├── main_runtime.rs
└── main.rs

```

## AUTOSAR 적용 범위 & VFB 중심 접근
- VFB: RTE DTO와 Broadcast 채널로 SW 컴포넌트 간 인터페이스를 정형화합니다.
- BSW: `ecu_abs_cam`, `ecu_abs_ultrasonic`, `ecu_abs_pwm`, `ecu_abs_com`, `ecu_abs_imu`가 센서·액추에이터·텔레메트리를 담당합니다.
- MCAL: OS 디바이스 드라이버(`libcamera`, `i2cdev`, `pwm_pca9685`)를 대체 활용합니다.

## 소프트웨어 컴포넌트 (SWC) 개요
- Vision_LaneFollowing (`src/asw/vs_lane.rs`): 전처리 → 투시 변환 → 슬라이딩 윈도우 → 조향각 계산. 칼만 필터 옵션 및 프레임 스로틀을 지원합니다.
- Vision_TrafficLight (`src/asw/vs_trafficlight.rs`): HSV + 모폴로지 + DBSCAN으로 신호색을 추정하고 디텍션 간격을 캘리브레이션합니다.
- UltraSonic_ForwardCollision (`src/asw/forwardcollision_ultrasonic.rs`): 거리 임계값 기반 장애물 이벤트 생성.
- ADAS Control (`src/asw/adas_cod.rs`): 차선 각도를 비례 제어 + `max_servo_delta_deg` 레이트 제한으로 서보 명령을 만들고, 초음파/신호등/거리 임계값을 통합한 정지·감속·순항 상태 머신을 구동합니다. 황색·소등 신호나 근접 거리는 감속 모드로 전환합니다.
- ADAS Localization & Path (`src/asw/adas_localization.rs`, `src/asw/adas_path_local.rs`, `src/asw/adas_path_global.rs`): 글로벌 맵과 로컬 궤적 계획을 위한 플레이스홀더입니다.


### RTE 채널 흐름
- 카메라: `raw_tx → processed_tx → bird_eye_tx / lane_angle_tx`, `raw_tx → traffic_light_tx`
- 초음파: `raw_tx → obstacle_tx`
- 제어: `servo_tx`, `dc_motor_tx`
- 텔레메트리/IMU: `com.telemetry_tx`(TCP 원시 페이로드) → `imu.raw_tx` → `imu.parsed_tx`

Broadcast 채널을 활용하여 각 Task가 비동기적으로 데이터를 주고받으며, `tokio` 런타임이 전체 파이프라인을 구성합니다.

## 주요 구성 요소
**주요 구성 요소**
- 카메라 파이프라인 (`src/asw/vs_lane.rs`, `src/asw/lib/vs_lane_lib.rs`): 전처리 → Bird's‑eye → 슬라이딩 윈도우 → 조향각. 칼만 필터 옵션.
- 신호등 인지 (`src/asw/vs_trafficlight.rs`, `src/asw/lib/vs_trafficlight_lib.rs`): HSV → 모폴로지 → DBSCAN → 색 판단.
- 초음파 장애물 감지 (`src/asw/forwardcollision_ultrasonic.rs`): 임계거리로 장애물 이벤트 생성.
- ECU Abstraction (`src/bsw/ecu_abs_*.rs`): 카메라/초음파/PCA9685/USB-TCP Gateway/IMU 파서를 묶어 하드웨어 I/O를 담당합니다.
- PCA9685 유틸 (`src/bsw/lib/pwm_lib.rs`): 서보/모터 변환 및 제어 유틸.
- USB/TCP IMU Gateway (`src/bsw/ecu_abs_com.rs`): iPhone ARKit 텔레메트리를 TCP 길이 프레이밍으로 수신하고 RTE 브로드캐스트로 배포합니다.
- IMU Protobuf Decoder (`src/bsw/ecu_abs_imu.rs`, `src/bsw/lib/imu_proto.rs`): Swift 앱이 보낸 protobuf payload를 파싱해 `DtoImu`로 변환하고 오일러 각까지 계산합니다.
- ADAS 제어 러너블 (`src/asw/adas_cod.rs`): 차선 각도를 비례 + 레이트 제한으로 서보에 반영하고, 초음파·신호등·거리 임계값에 따라 정지/감속/순항 상태 머신으로 DC 모터를 제어합니다.
- 런타임/프리뷰 (`src/main_runtime.rs`, `src/util/preview_*`): SDL2 기반 다중 창 프리뷰(ESC/창 닫기 종료).

### 카메라 해상도 & 캡처 모드
- 기본 해상도는 VGA 640×480(`LaneCalibrationPreset::Vga640x480`)입니다.
- 샘플 영상 `video/challenge.mp4`(16:9)를 사용하는 경우, 아래 스크립트로 중앙 크롭 후 640×480으로 리사이즈한 파일을 생성해 동일 조건에서 테스트할 수 있습니다.

```bash
python tools/resize_video.py --input video/challenge.mp4 \
    --output video/challenge_640x480.mp4
```

## 하드웨어 구성
- Raspberry Pi 4B (64-bit OS)
- RC 카 섀시 및 DC 모터, 스티어링 서보
- 카메라 모듈 (ov5647)
- HC-SR04 초음파 센서 
- PCA9685 PWM 보드 + I2C 배선
- 보조 전원 (모터 전용 배터리, 라즈베리파이 전원)


### 개발 환경 & Docker 개발 컨테이너
- `docker/dockerfile`은 멀티스테이지로 OpenCV/libcamera 등 의존성을 ARM64 타깃으로 빌드합니다.
- 최종 이미지는 런타임 라이브러리와 Rust toolchain만 포함해 경량화합니다.
- 예시 명령:
  ```bash
  docker-buildx build --platform linux/arm64 -t sangho007/rustkpi:latest --push .
  ```

### 크로스 컴파일 (라즈베리파이용)
- `rustup target add aarch64-unknown-linux-gnu`
- `gcc-aarch64-linux-gnu` 등 툴체인 설치 후:
- `cargo build --release --target aarch64-unknown-linux-gnu`

## 실행 방법
- 데스크톱(샘플 비디오) 기본 실행:
  - `cargo run --release`
  - SDL2 프리뷰 창이 뜨며 ESC 또는 창 닫기로 종료합니다.
- 실제 카메라(libcamera) 사용:
  - 캡처 모드는 `CameraCalibration::default().use_libcamera`에 의해 결정됩니다(현재 기본 false).
  - 필요 시 코드의 기본값(또는 HD 프리셋)에서 `use_libcamera = true`로 전환하세요.

## 의존성(로컬 실행)
- 시스템 패키지: OpenCV 런타임, SDL2 개발 패키지(예: Ubuntu `libsdl2-dev`)
- Rust 크레이트: `tokio`, `opencv`, `sdl2`, `rayon`, `dbscan`, `hc-sr04`, `linux-embedded-hal`, `pwm-pca9685`, `prost`

## Rust vs C/C++ 비교 계획 
- **안전성:** 메모리 안전, 데이터 레이스 방지
- **성능:** 파이프라인 레이턴시, CPU/GPU 사용량
- **개발 경험:** 빌드 체인, 디버깅, 크로스 컴파일 난이도
