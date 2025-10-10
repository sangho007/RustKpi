use std::sync::Arc;
use opencv::core::Mat;

#[derive(Debug, Clone)]
pub enum VfbEvent {
    // Cam
    CamRawEvent(Arc<DtoCamRaw>),
    CamProcessedEvent(Arc<DtoCamProcessed>),
    CamLaneAngleEvent(Arc<DtoCamLaneAngle>),
    CamCamBirdEyeViewEvent(Arc<DtoCamBirdEyeView>),
    // UltraSonic
    UltraSonicRawEvent(Arc<DtoUltraSonicRaw>),
    // Servo
    ServoCtrlEvent(Arc<DtoServoCtrl>),
    // DcMotor
    DcMotorCtrlEvent(Arc<DtoDcMotorCtrl>),
}

#[derive(Debug)]
pub struct DtoCamRaw {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl DtoCamRaw {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}

#[derive(Debug)]
pub struct DtoCamProcessed {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl DtoCamProcessed {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}


// 이 구조체는 Mat이 없으므로 수정할 필요가 없습니다.
// f64, u32는 크기가 작아 clone() 비용이 매우 저렴합니다.
#[derive(Debug, Clone)]
pub struct DtoCamLaneAngle {
    pub angle: f64,
    pub alive_cnt:u32,
}

impl DtoCamLaneAngle {
    pub fn new(angle: f64, alive_cnt: u32) -> Self {
        Self { angle, alive_cnt }
    }
}

#[derive(Debug)]
pub struct DtoCamBirdEyeView {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl DtoCamBirdEyeView {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}

#[derive(Debug, Clone)]
pub struct DtoUltraSonicRaw {
    pub distance: f32,
    pub alive_cnt:u32,
}

impl DtoUltraSonicRaw {
    pub fn new(distance: f32, alive_cnt: u32) -> Self {
        Self { distance, alive_cnt }
    }
}

#[derive(Debug, Clone)]
pub struct DtoServoCtrl {
    pub channel: u8,    // 제어할 서보 채널 (예: 0, 1, 2)
    pub angle: u32,     // 서보의 목표 각도 (0.0 ~ 180.0)
}

impl DtoServoCtrl {
    pub fn new(channel: u8, angle: u32) -> Self {
        Self { channel, angle }
    }
}

#[derive(Debug, Clone)]
pub struct DtoDcMotorCtrl {
    pub direction: u32,     // 서보의 목표 각도 (0.0 ~ 180.0)
    pub speed: u32,
    pub alive_cnt: u32,
}






