//! RTE(Runtime Environment) 계층에서 교환하는 데이터 객체(Data Transfer Object) 정의.
//! 각 DTO는 채널을 통해 전달되는 메시지 구조를 명시해 계층 간 의존성을 줄인다.

use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::calibration::{LocalizationLane, LocalizationMapId};
use crate::rte::lib::camera_lib;
pub use crate::rte::lib::camera_lib::{CameraBuffer, ColorFormat};
use opencv::core::Mat;
use std::sync::Arc;

#[derive(Debug)]
/// 카메라 RAW 프레임 데이터와 메타데이터.
pub struct DtoCamRaw {
    pub buffer: Arc<CameraBuffer>,
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub bytes_per_pixel: usize,
    pub alive_cnt: u32,
    pub color_format: ColorFormat,
}

impl DtoCamRaw {
    /// 새 RAW 프레임 DTO를 생성한다.
    pub fn new(
        buffer: Arc<CameraBuffer>,
        width: u32,
        height: u32,
        stride: usize,
        bytes_per_pixel: usize,
        alive_cnt: u32,
        color_format: ColorFormat,
    ) -> Self {
        Self {
            buffer,
            width,
            height,
            stride,
            bytes_per_pixel,
            alive_cnt,
            color_format,
        }
    }

    /// 내부 버퍼를 참조하는 OpenCV `Mat` 뷰를 생성한다.
    pub fn as_mat_view(&self) -> opencv::Result<Mat> {
        camera_lib::mat_from_buffer(
            self.buffer.as_ref(),
            self.width,
            self.height,
            self.bytes_per_pixel,
            self.stride,
        )
    }

    /// 색상 포맷이 다르더라도 BGR `Mat`으로 변환해 반환한다.
    pub fn as_bgr(&self) -> opencv::Result<Mat> {
        let base = self.as_mat_view()?;
        camera_lib::ensure_bgr(&base, self.color_format)
    }

    /// 과거 인터페이스와의 호환을 위해 `as_bgr_mat` 별칭을 유지한다.
    pub fn as_bgr_mat(&self) -> opencv::Result<Mat> {
        self.as_bgr()
    }
}

#[derive(Debug)]
/// 전처리된 회색조 프레임과 메타데이터.
pub struct DtoCamProcessed {
    pub img: Arc<Mat>,
    pub width: u32,
    pub height: u32,
    pub alive_cnt: u32,
}

impl DtoCamProcessed {
    /// 새로운 전처리 프레임 DTO를 생성한다.
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self {
            img,
            width,
            height,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// 차선 각도(조향각) 값과 alive 카운터.
pub struct DtoCamLaneAngle {
    pub angle: f64,
    pub alive_cnt: u32,
}

impl DtoCamLaneAngle {
    /// 새 차선 각도 DTO를 생성한다.
    pub fn new(angle: f64, alive_cnt: u32) -> Self {
        Self { angle, alive_cnt }
    }
}

#[derive(Debug)]
/// 버드아이(투시 변환) 영상과 메타데이터.
pub struct DtoCamBirdEyeView {
    pub img: Arc<Mat>,
    pub width: u32,
    pub height: u32,
    pub alive_cnt: u32,
}

impl DtoCamBirdEyeView {
    /// 버드아이 뷰 DTO를 생성한다.
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self {
            img,
            width,
            height,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// 초음파 센서에서 측정한 거리값.
pub struct DtoUltraSonicRaw {
    pub distance: f32,
    pub alive_cnt: u32,
}

impl DtoUltraSonicRaw {
    /// 새 초음파 거리 DTO를 생성한다.
    pub fn new(distance: f32, alive_cnt: u32) -> Self {
        Self {
            distance,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// 서보 모터 제어 명령 DTO.
pub struct DtoServoCtrl {
    pub channel: u8, // 제어할 서보 채널 인덱스
    pub angle: u32,  // 목표 각도(도 단위)
}

impl DtoServoCtrl {
    /// 서보 채널과 목표 각도를 지정해 생성한다.
    pub fn new(channel: u8, angle: u32) -> Self {
        Self { channel, angle }
    }
}

#[derive(Debug, Clone)]
/// DC 모터 제어 명령 DTO.
pub struct DtoDcMotorCtrl {
    pub direction: u32, // 0=정지, 1=정방향, 2=역방향
    pub speed: u32,
    pub alive_cnt: u32,
}

impl DtoDcMotorCtrl {
    /// 방향, 속도, alive 카운터를 포함한 DC 모터 명령을 생성한다.
    pub fn new(direction: u32, speed: u32, alive_cnt: u32) -> Self {
        Self {
            direction,
            speed,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// 초음파 기반 장애물 감지 결과.
pub struct DtoUltraSonicObstacle {
    pub detected: bool,
    pub alive_cnt: u32,
}

impl DtoUltraSonicObstacle {
    /// 장애물 검출 여부와 alive 카운터를 지정해 생성한다.
    pub fn new(detected: bool, alive_cnt: u32) -> Self {
        Self {
            detected,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// TCP를 통해 수신한 바이너리 텔레메트리 데이터.
pub struct DtoTcpTelemetry {
    pub payload: Arc<Vec<u8>>,
    pub message_size: usize,
    pub alive_cnt: u32,
}

impl DtoTcpTelemetry {
    /// 페이로드 벡터를 소유권과 함께 DTO로 래핑한다.
    pub fn new(payload: Vec<u8>, alive_cnt: u32) -> Self {
        let message_size = payload.len();
        Self {
            payload: Arc::new(payload),
            message_size,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone, Default)]
/// IMU 데이터 헤더 정보.
pub struct DtoImuHeader {
    pub stamp_ns: u64,
    pub dt_ns: u64,
    pub seq: u64,
    pub session_id: Option<String>,
    pub clock_domain: Option<String>,
    pub frame_id: Option<String>,
    pub child_frame_id: Option<String>,
}

#[derive(Debug, Clone, Default)]
/// IMU 추적 상태 요약.
pub struct DtoImuStatus {
    pub tracking: Option<String>,
    pub tracking_confidence: Option<f64>,
    pub num_features: Option<u64>,
    pub status_reason: Option<String>,
    pub flags: Vec<String>,
}

#[derive(Debug, Clone, Default)]
/// 3D 위치/자세 정보를 담는 구조체.
pub struct DtoImuPose {
    pub position_world: Option<[f64; 3]>,
    pub orientation_quat: Option<[f64; 4]>,
    pub orientation_yaw_roll_pitch: Option<[f64; 3]>,
    pub position_cov: Vec<f64>,
    pub orientation_cov: Vec<f64>,
    pub valid: Option<bool>,
}

#[derive(Debug, Clone, Default)]
/// 속도 측정 정보.
pub struct DtoImuVelocity {
    pub world: Option<[f64; 3]>,
    pub source: Option<String>,
    pub covariance: Vec<f64>,
    pub valid: Option<bool>,
}

#[derive(Debug, Clone, Default)]
/// 가속도 측정 정보.
pub struct DtoImuAcceleration {
    pub body_no_gravity: Option<[f64; 3]>,
    pub world: Option<[f64; 3]>,
    pub source: Option<String>,
    pub covariance: Vec<f64>,
    pub valid: Option<bool>,
}

#[derive(Debug, Clone, Default)]
/// 자이로(각속도) 측정 정보.
pub struct DtoImuGyro {
    pub body: Option<[f64; 3]>,
    pub source: Option<String>,
    pub bias: Option<[f64; 3]>,
    pub covariance: Vec<f64>,
    pub valid: Option<bool>,
}

#[derive(Debug, Clone)]
/// IMU 전체 패킷을 한 번에 전달하는 DTO.
pub struct DtoImu {
    pub header: DtoImuHeader,
    pub status: Option<DtoImuStatus>,
    pub pose: Option<DtoImuPose>,
    pub velocity: Option<DtoImuVelocity>,
    pub acceleration: Option<DtoImuAcceleration>,
    pub gyro: Option<DtoImuGyro>,
    pub alive_cnt: u32,
}

impl DtoImu {
    /// 세부 구성 요소를 수집해 새로운 IMU DTO를 생성한다.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        header: DtoImuHeader,
        status: Option<DtoImuStatus>,
        pose: Option<DtoImuPose>,
        velocity: Option<DtoImuVelocity>,
        acceleration: Option<DtoImuAcceleration>,
        gyro: Option<DtoImuGyro>,
        alive_cnt: u32,
    ) -> Self {
        Self {
            header,
            status,
            pose,
            velocity,
            acceleration,
            gyro,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone, Copy)]
/// 차량의 yaw 값을 도출한 데이터 소스.
pub enum LocalizationYawSource {
    ImuYaw,
    ImuPitch,
    ImuRoll,
    MotionEstimate,
}

#[derive(Debug, Clone)]
/// 2D 맵 좌표계에서 차량 위치/자세 정보를 전달하는 DTO.
pub struct DtoLocalizationState {
    pub map_id: LocalizationMapId,
    pub lane: LocalizationLane,
    pub position_map_xy: [f64; 2],
    pub displacement_imu_xyz: [f64; 3],
    pub yaw_rad: f64,
    pub yaw_source: LocalizationYawSource,
    pub motion_heading_rad: Option<f64>,
    pub timestamp_ns: u64,
    pub imu_alive_cnt: u32,
}

impl DtoLocalizationState {
    /// 신규 로컬라이제이션 상태 DTO를 생성한다.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        map_id: LocalizationMapId,
        lane: LocalizationLane,
        position_map_xy: [f64; 2],
        displacement_imu_xyz: [f64; 3],
        yaw_rad: f64,
        yaw_source: LocalizationYawSource,
        motion_heading_rad: Option<f64>,
        timestamp_ns: u64,
        imu_alive_cnt: u32,
    ) -> Self {
        Self {
            map_id,
            lane,
            position_map_xy,
            displacement_imu_xyz,
            yaw_rad,
            yaw_source,
            motion_heading_rad,
            timestamp_ns,
            imu_alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// Localization 목적지 도착 여부를 전파하는 DTO.
pub struct DtoLocalizationArrival {
    /// 목적지 임계 거리 이내면 `true`.
    pub arrived: bool,
    /// 목적지까지의 직선 거리(m).
    pub distance_m: f64,
    /// Localization 샘플 타임스탬프(ns).
    pub timestamp_ns: u64,
    /// 도착 판정 alive 카운터.
    pub alive_cnt: u32,
}

impl DtoLocalizationArrival {
    pub fn new(arrived: bool, distance_m: f64, timestamp_ns: u64, alive_cnt: u32) -> Self {
        Self {
            arrived,
            distance_m,
            timestamp_ns,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
/// 신호등 감지 결과 DTO.
pub struct DtoTrafficLight {
    pub traffic_light_color: TrafficLightColor,
    pub alive_cnt: u32,
}

impl DtoTrafficLight {
    /// 감지된 색상과 alive 카운터로 DTO를 생성한다.
    pub fn new(traffic_light_color: TrafficLightColor, alive_cnt: u32) -> Self {
        Self {
            traffic_light_color,
            alive_cnt,
        }
    }
}
