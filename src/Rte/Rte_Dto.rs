use std::sync::Arc;
use opencv::core::Mat;

#[derive(Debug, Clone)]
pub enum VfbEvent {
    CamRawData(Arc<Dto_CamRaw>),
    CamProcessedData(Arc<Dto_CamProcessed>),
    CamLaneAngleData(Arc<Dto_CamLaneAngle>),
    CamCamBirdEyeViewData(Arc<Dto_CamBirdEyeView>),
}

#[derive(Debug)] // Clone을 제거하고 필요 시 수동으로 구현하거나 Arc::clone()을 사용합니다.
pub struct Dto_CamRaw {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl Dto_CamRaw {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}

#[derive(Debug)]
pub struct Dto_CamProcessed {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl Dto_CamProcessed {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}


// 이 구조체는 Mat이 없으므로 수정할 필요가 없습니다.
// f64, u32는 크기가 작아 clone() 비용이 매우 저렴합니다.
#[derive(Debug, Clone)]
pub struct Dto_CamLaneAngle {
    pub angle: f64,
    pub alive_cnt:u32,
}

impl Dto_CamLaneAngle {
    pub fn new(angle: f64, alive_cnt: u32) -> Self {
        Self { angle, alive_cnt }
    }
}

#[derive(Debug)]
pub struct Dto_CamBirdEyeView {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
    pub width: u32,
    pub height: u32,
    pub alive_cnt:u32,
}

impl Dto_CamBirdEyeView {
    pub fn new(img: Arc<Mat>, width: u32, height: u32, alive_cnt: u32) -> Self {
        Self { img, width, height, alive_cnt }
    }
}



