use std::sync::Arc;
use opencv::core::Mat;

#[derive(Debug, Clone)]
pub enum VfbEvent {
    CamRawData(Arc<DtoCamRaw>),
    CamProcessedData(Arc<DtoCamProcessed>),
    CamLaneAngleData(Arc<DtoCamLaneAngle>),
    CamCamBirdEyeViewData(Arc<DtoCamBirdEyeView>),
}

#[derive(Debug)] // Clone을 제거하고 필요 시 수동으로 구현하거나 Arc::clone()을 사용합니다.
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
    pub distance: f64,
    pub alive_cnt:u32,
}



