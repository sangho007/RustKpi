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

pub fn mat_from_buffer(
    buffer: &CameraBuffer,
    width: u32,
    height: u32,
    bytes_per_pixel: usize,
    stride: usize,
) -> opencv::Result<Mat> {
    let mat_type = match bytes_per_pixel {
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
            height as i32,
            width as i32,
            mat_type,
            buffer.as_ptr() as *mut c_void,
            stride,
        )
    }
}

pub fn ensure_bgr(mat: &Mat, format: ColorFormat) -> opencv::Result<Mat> {
    match format {
        ColorFormat::Bgr => Ok(mat.clone()),
        ColorFormat::Rgb => {
            let mut converted = Mat::default();
            imgproc::cvt_color(
                mat,
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
                mat,
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
                mat,
                &mut converted,
                imgproc::COLOR_GRAY2BGR,
                0,
                AlgorithmHint::ALGO_HINT_DEFAULT,
            )?;
            Ok(converted)
        }
    }
}
