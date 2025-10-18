# RustKpi

자율주행 RC 카 플랫폼에서 **Classic AUTOSAR** 구조를 모사하며, Rust 기반으로 ADAS 핵심 기능을 실험하는 프로젝트입니다. 차선 유지 주행, 앞차 인지 시 차선 변경, 신호등 대응, 돌발 장애물 대응까지 최소 기능을 구현하고, 추후 C/C++ 버전과 비교해 언어별 장단점을 분석하는 것이 목표입니다.

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
┌────────────────────────────────────┐
│   Application Software (ASW)       │  Lane/Vision, Traffic Light, Foward Collision
├────────────────────────────────────┤
│   Runtime Environment (RTE)        │  DTO, Broadcast Channel, Scheduling
├────────────────────────────────────┤
│   Basic Software (BSW)             │  Camera, Ultrasonic, PCA9685 Driver
└────────────────────────────────────┘
```

| 계층 | 주요 모듈 | 경로 |
| --- | --- | --- |
| ASW | `vs_lane`, `vs_trafficlight`, `uss_forwardcollision` | `src/asw/` |
| RTE | `rte_main`, `rte_dto` | `src/rte/` |
| BSW | `ecu_abs_cam`, `ecu_abs_ultrasonic`, `ecu_abs_pca9685`, `pca9685_lib` | `src/bsw/` |

```
src
├── asw
│   ├── lib
│   │   ├── forwardcollision_ultrasonic_lib.rs
│   │   ├── vs_lane_lib.rs
│   │   └── vs_trafficlight_lib.rs
│   ├── forwardcollision_ultrasonic.rs
│   ├── vs_lane.rs
│   └── vs_trafficlight.rs
├── bsw
│   ├── lib
│   │   ├── cam_lib.rs
│   │   ├── pwm_lib.rs
│   │   └── ultrasonic_lib.rs
│   ├── ecu_abs_cam.rs
│   ├── ecu_abs_pwm.rs
│   ├── ecu_abs_ultrasonic.rs
├── rte
│   ├── rte_dto.rs
│   └── rte_main.rs
└── main.rs

```

## AUTOSAR 적용 범위 & VFB 중심 접근
- **Virtual Functional Bus(VFB)**: RTE와 DTO를 통해 소프트웨어 컴포넌트 간 인터페이스를 정형화하고, 실제 MCAL 부재 상황에서도 기능 검증이 가능하도록 했습니다.
- **AUTOSAR 스택 단순화**: Classic AUTOSAR의 전체 스택을 구현하지 않고, 실험에 필요한 계층만 선별 적용했습니다. VFB와 BSW-ECU Abstraction에 집중하며, Complex Device Driver나 Diagnostics 등은 TODO로 남겨둡니다.
- **BSW 구성 방식**: `ecu_abs_*` 모듈은 라즈베리파이 하드웨어를 대상으로 직접 작성한 ECU Abstraction입니다. Linux 사용자 공간 드라이버를 래핑하여 HAL에 가까운 API를 제공합니다.
- **MCAL 처리 전략**: 실제 MCAL 드라이버는 포함되어 있지 않으며, 시스템 패키지로 설치되는 `libcamera`, `i2cdev`, `pwm_pca9685` 등 Linux 디바이스 드라이버를 MCAL 대체제로 사용합니다. 

## 소프트웨어 컴포넌트 (SWC) 개요
- **Vision_LaneFollowing (`src/asw/vs_lane.rs`)**: 카메라 입력을 받아 전처리, 차선 검출, 조향각 계산까지 수행하며 VFB를 통해 조향 명령을 게시합니다.
- **Vision_TrafficLight (`src/asw/vs_trafficlight.rs`)**: 신호등 색상/형태를 인식하여 차량 정지/출발 이벤트를 결정합니다. 색상 분류기 및 상태 머신 보강이 TODO입니다.
- **UltraSonic_ForwardCollision (`src/asw/uss_forwardcollision.rs`)**: 초음파 센서 데이터를 장애물 DTO로 변환하고, 위험 거리 계산 후 제동 요청을 올립니다.
- **Adas_ControlFusion (`src/asw` 예정)**: Lane, TrafficLight, FCA 결과를 통합하여 스티어링/구동 명령을 생성하는 통합 제어 SWC는 설계 단계입니다.


### RTE 채널 흐름
- 카메라: `raw_tx → processed_tx → bird_eye_tx → lane_angle_tx`
- 초음파: `raw_tx → obstacle_tx`
- 제어: `control_tx → servo_tx, dc_motor_tx`

Broadcast 채널을 활용하여 각 Task가 비동기적으로 데이터를 주고받으며, `tokio` 런타임이 전체 파이프라인을 구성합니다.

## 주요 구성 요소
- **카메라 파이프라인 (`src/asw/vs_lane.rs`, `lib/vs_lane_lib.rs`)**  
  전처리 → 버즈아이 변환 → 슬라이딩 윈도우 → 조향각 산출. Kalman Filter 옵션 지원.
- **신호등 인지 (`src/asw/vs_trafficlight.rs`)**  
  OpenCV 기반 색상/형태 필터링으로 신호 상태 추정. (세부 구현 진행 중)
- **초음파 장애물 감지 (`src/asw/uss_obstacle.rs`)**  
  Raw 데이터를 장애물 DTO로 변환하여 FCA 로직에 전달.
- **ECU Abstraction (`src/bsw/ecu_abs_*.rs`)**  
  카메라/초음파/PCA9685를 대상으로 한 자체 작성 BSW 계층. Linux 드라이버 호출을 캡슐화하여 AUTOSAR BSW 패턴에 맞춘 API를 제공합니다.
- **PCA9685 제어 (`src/bsw/lib/pca9685_lib.rs`)**  
  서보 각도 → PWM 변환, DC 모터 속도/방향 제어, 긴급 정지 API 제공.
- **통합 실행 (`src/main.rs`)**  
  시스템 초기화, Task 생성, 디버깅용 GUI (OpenCV HighGUI).

## 하드웨어 구성
- Raspberry Pi 4B (64-bit OS)
- RC 카 섀시 및 DC 모터, 스티어링 서보
- 카메라 모듈 (ov5647)
- HC-SR04 초음파 센서 
- PCA9685 PWM 보드 + I2C 배선
- 보조 전원 (모터 전용 배터리, 라즈베리파이 전원)


### 개발 환경 & Docker 개발 컨테이너
- `docker/dockerfile`은 **2단계 멀티스테이지**로 구성되어, 빌더 단계에서 OpenCV 4.11.0, libcamera(next), rpicam-apps, kmsxx, Python 드라이버까지 라즈베리파이용으로 컴파일합니다.
- 최종 이미지에는 런타임 라이브러리와 Rust toolchain, LLVM 17, Python 패키지만 포함하여 배포 환경을 최대한 가볍게 유지합니다.
- 빌드는 `docker-buildx`를 이용해 `linux/arm64` 타깃으로 수행하며, 예시 명령은 다음과 같습니다.
  ```bash
  docker-buildx build --platform linux/arm64 -t sangho007/rustkpi:latest --push .
  ```

### 크로스 컴파일 (라즈베리파이용)
1. `rustup target add aarch64-unknown-linux-gnu`
2. 크로스 컴파일 도구체인 설정 (`gcc-aarch64-linux-gnu` 등)
3. `cargo build --release --target aarch64-unknown-linux-gnu`

## 향후 개선점
- [ ] Trajectory Planning 및 Lane Change 의사결정 SWC 구현
- [ ] LCA용 주변 차량 인지 로직 (추가 센서 연동)
- [ ] 2D 데이터맵 기반 신호등 인지 성능 향상 및 다중 교차로 시나리오 지원
- [ ] FCA 대응 시 감속 프로파일 및 경고 HMI 모듈 구현
- [ ] AUTOSAR 서비스 계층 (Diagnostics, NvM) 최소 기능 도입
- [ ] C/C++ 포팅 후 성능·안전성 비교 리포트 작성
- [ ] Posix 지원 Yocto 포팅 

## Rust vs C/C++ 비교 계획 
- **안전성:** 메모리 안전, 데이터 레이스 방지
- **성능:** 파이프라인 레이턴시, CPU/GPU 사용량
- **개발 경험:** 빌드 체인, 디버깅, 크로스 컴파일 난이도
