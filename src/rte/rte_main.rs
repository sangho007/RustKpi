//! RTE 채널 초기화 및 공유 구조체 정의.
//! 각 ECU/ASW 태스크가 사용할 브로드캐스트 송신자를 생성한다.

use crate::rte::rte_dto::{
    DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed, DtoCamRaw, DtoDcMotorCtrl, DtoImu,
    DtoServoCtrl, DtoTcpTelemetry, DtoTrafficLight, DtoUltraSonicObstacle, DtoUltraSonicRaw,
};
use std::sync::Arc;
use tokio::sync::broadcast;

pub type CamRawSender = broadcast::Sender<Arc<DtoCamRaw>>;
pub type CamProcessedSender = broadcast::Sender<Arc<DtoCamProcessed>>;
pub type CamBirdEyeSender = broadcast::Sender<Arc<DtoCamBirdEyeView>>;
pub type CamLaneAngleSender = broadcast::Sender<Arc<DtoCamLaneAngle>>;
pub type TrafficLightSender = broadcast::Sender<Arc<DtoTrafficLight>>;
pub type UltraRawSender = broadcast::Sender<Arc<DtoUltraSonicRaw>>;
pub type UltraObstacleSender = broadcast::Sender<Arc<DtoUltraSonicObstacle>>;
pub type ServoCtrlSender = broadcast::Sender<Arc<DtoServoCtrl>>;
pub type DcMotorCtrlSender = broadcast::Sender<Arc<DtoDcMotorCtrl>>;
pub type TcpRawSender = broadcast::Sender<Arc<DtoTcpTelemetry>>;
pub type ImuParsedSender = broadcast::Sender<Arc<DtoImu>>;

const CAM_RAW_CAPACITY: usize = 2;
const CAM_PROCESSED_CAPACITY: usize = 6;
const CAM_BIRD_EYE_CAPACITY: usize = 4;
const CAM_LANE_ANGLE_CAPACITY: usize = 8;
const TRAFFIC_LIGHT_CAPACITY: usize = 8;
const ULTRA_RAW_CAPACITY: usize = 8;
const ULTRA_OBSTACLE_CAPACITY: usize = 8;
const SERVO_CTRL_CAPACITY: usize = 16;
const DC_CTRL_CAPACITY: usize = 16;
const TCP_RAW_CAPACITY: usize = 16;
const IMU_PARSED_CAPACITY: usize = 16;

#[derive(Clone)]
/// 카메라 처리 파이프라인에 필요한 브로드캐스트 송신자 묶음.
pub struct CameraChannels {
    pub raw_tx: CamRawSender,
    pub processed_tx: CamProcessedSender,
    pub bird_eye_tx: CamBirdEyeSender,
    pub lane_angle_tx: CamLaneAngleSender,
    pub traffic_light_tx: TrafficLightSender,
}

#[derive(Clone)]
/// 초음파 센서 관련 송신자.
pub struct UltrasonicChannels {
    pub raw_tx: UltraRawSender,
    pub obstacle_tx: UltraObstacleSender,
}

#[derive(Clone)]
/// 서보 및 DC 모터 제어 채널.
pub struct ControlChannels {
    pub servo_tx: ServoCtrlSender,
    pub dc_motor_tx: DcMotorCtrlSender,
}

#[derive(Clone)]
/// TCP 텔레메트리(USB 터널링) 채널.
pub struct TcpChannels {
    pub telemetry_tx: TcpRawSender,
}

#[derive(Clone)]
/// IMU 원시/파싱 채널 세트.
pub struct ImuChannels {
    pub raw_tx: TcpRawSender,
    pub parsed_tx: ImuParsedSender,
}

#[derive(Clone)]
/// 전체 RTE 채널 묶음.
pub struct RteChannels {
    pub camera: CameraChannels,
    pub ultrasonic: UltrasonicChannels,
    pub control: ControlChannels,
    pub com: TcpChannels,
    pub imu: ImuChannels,
}

/// RTE 시스템 초기화 결과.
pub struct RteSystem {
    pub channels: RteChannels,
}

/// 모든 브로드캐스트 채널을 초기화하고 공유 핸들을 반환한다.
pub fn init() -> RteSystem {
    let (cam_raw_tx, _) = broadcast::channel(CAM_RAW_CAPACITY);
    let (cam_processed_tx, _) = broadcast::channel(CAM_PROCESSED_CAPACITY);
    let (cam_bird_eye_tx, _) = broadcast::channel(CAM_BIRD_EYE_CAPACITY);
    let (cam_lane_angle_tx, _) = broadcast::channel(CAM_LANE_ANGLE_CAPACITY);
    let (traffic_light_tx, _) = broadcast::channel(TRAFFIC_LIGHT_CAPACITY);

    let (ultra_raw_tx, _) = broadcast::channel(ULTRA_RAW_CAPACITY);
    let (ultra_obstacle_tx, _) = broadcast::channel(ULTRA_OBSTACLE_CAPACITY);

    let (servo_ctrl_tx, _) = broadcast::channel(SERVO_CTRL_CAPACITY);
    let (dc_motor_ctrl_tx, _) = broadcast::channel(DC_CTRL_CAPACITY);
    let (imu_raw_tx, _) = broadcast::channel(TCP_RAW_CAPACITY);
    let (imu_parsed_tx, _) = broadcast::channel(IMU_PARSED_CAPACITY);

    let camera = CameraChannels {
        raw_tx: cam_raw_tx,
        processed_tx: cam_processed_tx,
        bird_eye_tx: cam_bird_eye_tx,
        lane_angle_tx: cam_lane_angle_tx,
        traffic_light_tx,
    };

    let ultrasonic = UltrasonicChannels {
        raw_tx: ultra_raw_tx,
        obstacle_tx: ultra_obstacle_tx,
    };

    let control = ControlChannels {
        servo_tx: servo_ctrl_tx,
        dc_motor_tx: dc_motor_ctrl_tx,
    };
    let com = TcpChannels {
        telemetry_tx: imu_raw_tx.clone(),
    };
    let imu = ImuChannels {
        raw_tx: imu_raw_tx,
        parsed_tx: imu_parsed_tx,
    };

    RteSystem {
        channels: RteChannels {
            camera,
            ultrasonic,
            control,
            com,
            imu,
        },
    }
}
