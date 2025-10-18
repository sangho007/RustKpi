pub mod picamera_capture {
    use opencv::core::{Mat, Mat_AUTO_STEP, CV_8UC3};
    use pyo3::exceptions::PyRuntimeError;
    use pyo3::prelude::*;
    use pyo3::types::PyDict;
    use numpy::PyArrayMethods;
    use std::ffi::c_void;

    pub const CAM_MODE: i32 = 1;

        // Python picamera2 객체를 감싸는 Rust 구조체
    pub struct PiCamera2 {
        py_instance: Py<PyAny>,
        width: u32,
        height: u32,
    }

    impl PiCamera2 {
        /// 새로운 PiCamera2 인스턴스를 생성하고 초기화합니다.
        pub fn new(width: u32, height: u32) -> PyResult<Self> {
            Python::with_gil(|py| {
                let picamera2_module = PyModule::import(py, "picamera2.picamera2")?;
                let picam2 = picamera2_module.getattr("Picamera2")?.call0()?;

                let config_args = PyDict::new(py);
                let main_config = PyDict::new(py);
                main_config.set_item("size", (width, height))?;
                // OpenCV Mat은 기본적으로 BGR 순서를 사용하므로 BGR888로 요청합니다.
                main_config.set_item("format", "BGR888")?;
                config_args.set_item("main", main_config)?;

                let config = picam2.call_method("create_preview_configuration", (), Some(&config_args))?;
                picam2.call_method1("configure", (config,))?;
                picam2.call_method0("start")?;

                // 카메라 안정화를 위해 잠시 대기
                let time = PyModule::import(py, "time")?;
                time.call_method1("sleep", (1.0,))?;

                println!("[bsw] Picamera2 초기화 완료 ({}x{})", width, height);

                Ok(Self {
                    py_instance: picam2.into(),
                    width,
                    height,
                })
            })
        }

            /// 카메라에서 한 프레임을 캡처하여 opencv::Mat으로 변환합니다.
        pub fn capture_frame(&self) -> PyResult<Mat> {
            Python::with_gil(|py| {
                // `capture_array()`를 호출하여 NumPy 배열 객체를 가져옵니다.
                let np_array_obj = self.py_instance.call_method0(py, "capture_array")?;

                // PyAny 객체를 PyArray3<u8> 타입으로 안전하게 다운캐스트합니다.
                let np_array = np_array_obj.downcast_bound::<numpy::PyArray3<u8>>(py)?;
                let readonly_array = np_array.readonly(); // Now it has a name and a longer life
                let data = readonly_array.as_slice()?;    // Borrowing from it is now safe

                // `unsafe` 블록 안에서 데이터에 대한 뷰(view)로 Mat을 생성합니다.
                // 이 Mat은 데이터를 소유하지 않습니다.
                let mat_view = unsafe {
                    Mat::new_rows_cols_with_data_unsafe(
                        self.height as i32,
                        self.width as i32,
                        CV_8UC3,
                        data.as_ptr() as *mut c_void, // const u8* -> mut c_void* 캐스팅
                        Mat_AUTO_STEP,
                    )
                }
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to create Mat from NumPy array: {}", e)))?; // <-- '?'를 사용해 Result를 처리하고 Mat을 가져옵니다.

                // mat_view.clone()은 데이터를 깊은 복사(deep copy)하여
                // 데이터 소유권을 가진 새로운 Mat을 생성합니다.
                Ok(mat_view.clone())
            })
        }

    }

        // PiCamera2가 Drop될 때 Python 객체를 안전하게 정지시킵니다.
    impl Drop for PiCamera2 {
        fn drop(&mut self) {
            let _ = Python::with_gil(|py| self.py_instance.call_method0(py, "stop"));
            println!("[bsw] Picamera2 정지.");
        }
    }
}
use opencv::core::Mat;
use opencv::prelude::{MatTraitConst, VideoCaptureTrait, VideoCaptureTraitConst};
use opencv::{videoio, Result};

/// 프레임을 읽어오는 동작을 추상화하는 Trait
pub trait FrameCapture: Send {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool>;
}

// 기존 `videoio::VideoCapture`에 대해 Trait 구현
impl FrameCapture for videoio::VideoCapture {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool> {
        VideoCaptureTrait::read(self, frame)
    }
}

// 새로 만든 `PiCamera2`에 대해 Trait 구현
impl FrameCapture for picamera_capture::PiCamera2 {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool> {
        match self.capture_frame() {
            Ok(captured_frame) => {
                if captured_frame.empty() {
                    Ok(false)
                } else {
                    *frame = captured_frame;
                    Ok(true)
                }
            }
            Err(e) => {
                // pyo3::PyErr를 opencv::Error로 변환하기 위해 에러 메시지로 새로 생성
                eprintln!("[bsw] Picamera2 프레임 캡처 실패: {:?}", e);
                Err(opencv::Error::new(
                    opencv::core::StsError,
                    format!("Picamera2 Error: {}", e),
                ))
            }
        }
    }
}

impl FrameCapture for libcamera_capture::LibcameraCapture {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool> {
        self.capture_into(frame)
    }
}
pub mod libcamera_capture {
    use opencv::core::{AlgorithmHint, Mat, CV_8UC3};
    use opencv::imgproc;
    use opencv::prelude::MatTrait;
    use opencv::{Error, Result};
    use std::cmp;
    use std::ffi::{c_char, c_void, CStr};
    use std::ptr::NonNull;

    #[repr(C)]
    struct LibcameraBridgeOpaque {
        _private: [u8; 0],
    }

    unsafe extern "C" {
        fn libcamera_bridge_create(
            width: u32,
            height: u32,
            fps: u32,
            out_stride: *mut u32,
            out_bpp: *mut u32,
            err_buf: *mut c_char,
            err_len: usize,
        ) -> *mut LibcameraBridgeOpaque;

        fn libcamera_bridge_capture(
            handle: *mut LibcameraBridgeOpaque,
            buffer: *mut u8,
            buffer_len: usize,
            out_size: *mut usize,
            timestamp_ns: *mut u64,
            err_buf: *mut c_char,
            err_len: usize,
        ) -> i32;

        fn libcamera_bridge_destroy(handle: *mut LibcameraBridgeOpaque);
    }

    const ERR_BUF_LEN: usize = 256;

    fn opencv_err(msg: impl Into<String>) -> Error {
        Error::new(opencv::core::StsError, msg.into())
    }

    pub struct LibcameraCapture {
        handle: NonNull<LibcameraBridgeOpaque>,
        width: u32,
        height: u32,
        stride: usize,
        bytes_per_pixel: usize,
        buffer: Vec<u8>,
    }

    impl LibcameraCapture {
        pub fn new(width: u32, height: u32, fps: u32) -> Result<Self> {
            let mut stride = 0u32;
            let mut bpp = 0u32;
            let mut err_buf = [0 as c_char; ERR_BUF_LEN];

            let handle = unsafe {
                libcamera_bridge_create(
                    width,
                    height,
                    fps,
                    &mut stride as *mut u32,
                    &mut bpp as *mut u32,
                    err_buf.as_mut_ptr(),
                    ERR_BUF_LEN,
                )
            };

            if handle.is_null() {
                let msg = unsafe { CStr::from_ptr(err_buf.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                let msg = if msg.is_empty() {
                    "Failed to initialize libcamera bridge".to_string()
                } else {
                    msg
                };
                return Err(opencv_err(msg));
            }

            let stride_usize = stride as usize;
            let bytes_per_pixel = cmp::max(1usize, bpp as usize);
            let buffer_len = stride_usize
                .checked_mul(height as usize)
                .ok_or_else(|| opencv_err("libcamera buffer size overflow"))?;
            let buffer = vec![0u8; buffer_len];

            Ok(Self {
                handle: unsafe { NonNull::new_unchecked(handle) },
                width,
                height,
                stride: stride_usize,
                bytes_per_pixel,
                buffer,
            })
        }

        pub fn capture_into(&mut self, frame: &mut Mat) -> Result<bool> {
            let mut out_size = 0usize;
            let mut timestamp_ns = 0u64;
            let mut err_buf = [0 as c_char; ERR_BUF_LEN];

            let rc = unsafe {
                libcamera_bridge_capture(
                    self.handle.as_ptr(),
                    self.buffer.as_mut_ptr(),
                    self.buffer.len(),
                    &mut out_size as *mut usize,
                    &mut timestamp_ns as *mut u64,
                    err_buf.as_mut_ptr(),
                    ERR_BUF_LEN,
                )
            };

            if rc != 0 {
                let msg = unsafe { CStr::from_ptr(err_buf.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                let msg = if msg.is_empty() {
                    format!("libcamera capture failed (code {})", rc)
                } else {
                    msg
                };
                return Err(opencv_err(msg));
            }

            if out_size == 0 {
                return Ok(false);
            }

            if self.bytes_per_pixel != 3 {
                return Err(opencv_err(format!(
                    "Unsupported pixel size from libcamera: {} bytes",
                    self.bytes_per_pixel
                )));
            }

            let mut rgb_mat = unsafe {
                Mat::new_rows_cols_with_data_unsafe(
                    self.height as i32,
                    self.width as i32,
                    CV_8UC3,
                    self.buffer.as_mut_ptr() as *mut c_void,
                    self.stride,
                )?
            };

            imgproc::cvt_color(
                &rgb_mat,
                frame,
                imgproc::COLOR_RGB2BGR,
                0,
                AlgorithmHint::ALGO_HINT_DEFAULT,
            )?;

            // Ensure we release the reference before the next capture.
            unsafe {
                rgb_mat.release()?;
            }

            Ok(true)
        }

        pub fn dimensions(&self) -> (u32, u32) {
            (self.width, self.height)
        }
    }

    impl Drop for LibcameraCapture {
        fn drop(&mut self) {
            unsafe {
                libcamera_bridge_destroy(self.handle.as_ptr());
            }
        }
    }

    unsafe impl Send for LibcameraCapture {}
}
