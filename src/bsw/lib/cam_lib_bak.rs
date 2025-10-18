pub mod picamera_capture {
    use opencv::core::{Mat, Mat_AUTO_STEP, CV_8UC3};
    use opencv::{prelude::*, videoio, Result};
    use pyo3::prelude::*;
    use pyo3::types::PyDict;
    use numpy::{PyArrayMethods};
    use pyo3::exceptions::PyRuntimeError;
    use std::ffi::c_void;
    use crate::bsw::lib::cam_lib;

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
// ===================================================================
// 2. 공통 인터페이스를 위한 Trait 정의 및 구현
// ===================================================================
use opencv::core::Mat;
use opencv::{prelude::*,videoio,Result};

/// 프레임을 읽어오는 동작을 추상화하는 Trait
pub trait FrameCapture: Send {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool>;
}

// 기존 `videoio::VideoCapture`에 대해 Trait 구현
impl FrameCapture for videoio::VideoCapture {
    fn read_frame(&mut self, frame: &mut Mat) -> Result<bool> {
        videoio::VideoCapture::read(self, frame)
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
