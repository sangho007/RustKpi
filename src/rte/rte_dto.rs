use crate::asw::lib::vs_trafficlight_lib::TrafficLightColor;
use opencv::core::{AlgorithmHint, CV_8UC1, CV_8UC3, CV_8UC4, Mat};
use opencv::imgproc;
use std::ffi::c_void;
use std::sync::Arc;

pub trait BufferRecycler: Send + Sync {
    fn recycle(&self, buffer: Vec<u8>);
}

pub struct CameraBuffer {
    data: Vec<u8>,
    recycler: Option<Arc<dyn BufferRecycler>>,
}

impl CameraBuffer {
    pub fn from_vec(data: Vec<u8>) -> Self {
        Self {
            data,
            recycler: None,
        }
    }

    pub fn from_vec_with_recycler(data: Vec<u8>, recycler: Arc<dyn BufferRecycler>) -> Self {
        Self {
            data,
            recycler: Some(recycler),
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.data.as_ptr()
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }
}

impl Drop for CameraBuffer {
    fn drop(&mut self) {
        if let Some(recycler) = self.recycler.as_ref() {
            let mut data = Vec::new();
            std::mem::swap(&mut data, &mut self.data);
            recycler.recycle(data);
        }
    }
}

impl std::fmt::Debug for CameraBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CameraBuffer")
            .field("len", &self.data.len())
            .finish()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorFormat {
    Bgr,
    Rgb,
    Rgba,
    Gray,
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
        let mat_type = match self.bytes_per_pixel {
            1 => CV_8UC1,
            3 => CV_8UC3,
            4 => CV_8UC4,
            other => {
                return Err(opencv::Error::new(
                    opencv::core::StsBadArg,
                    format!("Unsupported pixel size: {}", other),
                ));
            }
        };

        unsafe {
            Mat::new_rows_cols_with_data_unsafe(
                self.height as i32,
                self.width as i32,
                mat_type,
                self.buffer.as_ptr() as *mut c_void,
                self.stride,
            )
        }
    }

    pub fn as_bgr_mat(&self) -> opencv::Result<Mat> {
        let base = self.as_mat_view()?;
        match self.color_format {
            ColorFormat::Bgr => Ok(base),
            ColorFormat::Rgb => {
                let mut converted = Mat::default();
                imgproc::cvt_color(
                    &base,
                    &mut converted,
                    imgproc::COLOR_RGB2BGR,
                    0,
                    AlgorithmHint::ALGO_HINT_DEFAULT,
                )?;
                Ok(converted)
            }
            ColorFormat::Rgba => {
                let mut converted = Mat::default();
                imgproc::cvt_color(
                    &base,
                    &mut converted,
                    imgproc::COLOR_RGBA2BGR,
                    0,
                    AlgorithmHint::ALGO_HINT_DEFAULT,
                )?;
                Ok(converted)
            }
            ColorFormat::Gray => {
                let mut converted = Mat::default();
                imgproc::cvt_color(
                    &base,
                    &mut converted,
                    imgproc::COLOR_GRAY2BGR,
                    0,
                    AlgorithmHint::ALGO_HINT_DEFAULT,
                )?;
                Ok(converted)
            }
        }
    }
}

#[derive(Debug)]
pub struct DtoCamProcessed {
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
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

// 이 구조체는 Mat이 없으므로 수정할 필요가 없습니다.
// f64, u32는 크기가 작아 clone() 비용이 매우 저렴합니다.
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
    pub img: Arc<Mat>, // Mat -> Arc<Mat>
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
