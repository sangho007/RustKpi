use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use crate::rte::lib::camera_lib;
pub use crate::rte::lib::camera_lib::{BufferRecycler, CameraBuffer, ColorFormat};
use opencv::core::Mat;
use std::sync::Arc;

#[derive(Debug)]
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

    pub fn as_mat_view(&self) -> opencv::Result<Mat> {
        camera_lib::mat_from_buffer(
            self.buffer.as_ref(),
            self.width,
            self.height,
            self.bytes_per_pixel,
            self.stride,
        )
    }

    pub fn as_bgr_mat(&self) -> opencv::Result<Mat> {
        let base = self.as_mat_view()?;
        camera_lib::ensure_bgr(&base, self.color_format)
    }
}

#[derive(Debug)]
pub struct DtoCamProcessed {
    pub img: Arc<Mat>,
    pub width: u32,
    pub height: u32,
    pub alive_cnt: u32,
}

impl DtoCamProcessed {
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
pub struct DtoCamLaneAngle {
    pub angle: f64,
    pub alive_cnt: u32,
}

impl DtoCamLaneAngle {
    pub fn new(angle: f64, alive_cnt: u32) -> Self {
        Self { angle, alive_cnt }
    }
}

#[derive(Debug)]
pub struct DtoCamBirdEyeView {
    pub img: Arc<Mat>,
    pub width: u32,
    pub height: u32,
    pub alive_cnt: u32,
}

impl DtoCamBirdEyeView {
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
pub enum VfbEvent {
    // Cam
    CamRawEvent(Arc<DtoCamRaw>),
    CamProcessedEvent(Arc<DtoCamProcessed>),
    CamLaneAngleEvent(Arc<DtoCamLaneAngle>),
    CamBirdEyeViewEvent(Arc<DtoCamBirdEyeView>),
    CamTrafficLightEvent(Arc<DtoTrafficLight>),
    // UltraSonic
    UltraSonicRawEvent(Arc<DtoUltraSonicRaw>),
    UltraSonicObstacleDetectedEvent(Arc<DtoUltraSonicObstacle>),
    // Servo
    ServoCtrlEvent(Arc<DtoServoCtrl>),
    // DcMotor
    DcMotorCtrlEvent(Arc<DtoDcMotorCtrl>),
}

#[derive(Debug, Clone)]
pub struct DtoUltraSonicRaw {
    pub distance: f32,
    pub alive_cnt: u32,
}

impl DtoUltraSonicRaw {
    pub fn new(distance: f32, alive_cnt: u32) -> Self {
        Self {
            distance,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DtoServoCtrl {
    pub channel: u8, // 제어할 서보 채널 (예: 0, 1, 2)
    pub angle: u32,  // 서보의 목표 각도 (0.0 ~ 180.0)
}

impl DtoServoCtrl {
    pub fn new(channel: u8, angle: u32) -> Self {
        Self { channel, angle }
    }
}

#[derive(Debug, Clone)]
pub struct DtoDcMotorCtrl {
    pub direction: u32, // 서보의 목표 각도 (0.0 ~ 180.0)
    pub speed: u32,
    pub alive_cnt: u32,
}

impl DtoDcMotorCtrl {
    pub fn new(direction: u32, speed: u32, alive_cnt: u32) -> Self {
        Self {
            direction,
            speed,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DtoUltraSonicObstacle {
    pub detected: bool,
    pub alive_cnt: u32,
}

impl DtoUltraSonicObstacle {
    pub fn new(detected: bool, alive_cnt: u32) -> Self {
        Self {
            detected,
            alive_cnt,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DtoTrafficLight {
    pub traffic_light_color: TrafficLightColor,
    pub alive_cnt: u32,
}

impl DtoTrafficLight {
    pub fn new(traffic_light_color: TrafficLightColor, alive_cnt: u32) -> Self {
        Self {
            traffic_light_color,
            alive_cnt,
        }
    }
}
