use crate::rte::rte_dto::{CameraBuffer, ColorFormat};
use opencv::core::Mat;
use opencv::prelude::{MatTraitConst, MatTraitConstManual, VideoCaptureTrait};
use opencv::{Result, videoio};
use std::sync::Arc;

pub struct CapturedFrame {
    pub buffer: Arc<CameraBuffer>,
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub bytes_per_pixel: usize,
    pub color_format: ColorFormat,
}

impl CapturedFrame {
    pub fn new(
        buffer: Arc<CameraBuffer>,
        width: u32,
        height: u32,
        stride: usize,
        bytes_per_pixel: usize,
        color_format: ColorFormat,
    ) -> Self {
        Self {
            buffer,
            width,
            height,
            stride,
            bytes_per_pixel,
            color_format,
        }
    }
}

/// 프레임을 읽어오는 동작을 추상화하는 Trait
pub trait FrameCapture: Send {
    fn read_frame(&mut self) -> Result<Option<CapturedFrame>>;
}

// 기존 `videoio::VideoCapture`에 대해 Trait 구현
impl FrameCapture for videoio::VideoCapture {
    fn read_frame(&mut self) -> Result<Option<CapturedFrame>> {
        let mut frame = Mat::default();
        if !VideoCaptureTrait::read(self, &mut frame)? || frame.empty() {
            return Ok(None);
        }

        let width = frame.cols() as u32;
        let height = frame.rows() as u32;
        let bytes_per_pixel = frame.elem_size()? as usize;
        let stride = width as usize * bytes_per_pixel;
        let data = frame.data_bytes()?;
        let buffer = CameraBuffer::from_vec(data.to_vec());

        Ok(Some(CapturedFrame::new(
            Arc::new(buffer),
            width,
            height,
            stride,
            bytes_per_pixel,
            ColorFormat::Bgr,
        )))
    }
}

impl FrameCapture for libcamera_capture::LibcameraCapture {
    fn read_frame(&mut self) -> Result<Option<CapturedFrame>> {
        self.capture_frame()
    }
}
pub mod libcamera_capture {
    use super::CapturedFrame;
    use crate::rte::rte_dto::{BufferRecycler, CameraBuffer, ColorFormat};
    use opencv::{Error, Result};
    use std::cmp;
    use std::ffi::{CStr, c_char};
    use std::ptr::NonNull;
    use std::sync::{Arc, Mutex};

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

    struct BufferPool {
        buffer_len: usize,
        buffers: Mutex<Vec<Vec<u8>>>,
    }

    impl BufferPool {
        fn new(buffer_len: usize, preallocate: usize) -> Arc<Self> {
            let mut buffers = Vec::with_capacity(preallocate);
            for _ in 0..preallocate {
                let mut vec = Vec::with_capacity(buffer_len);
                unsafe {
                    vec.set_len(buffer_len);
                }
                buffers.push(vec);
            }

            Arc::new(Self {
                buffer_len,
                buffers: Mutex::new(buffers),
            })
        }

        fn acquire(&self) -> Vec<u8> {
            let mut guard = self.buffers.lock().unwrap();
            guard.pop().unwrap_or_else(|| {
                let mut vec = Vec::with_capacity(self.buffer_len);
                unsafe {
                    vec.set_len(self.buffer_len);
                }
                vec
            })
        }
    }

    impl BufferRecycler for BufferPool {
        fn recycle(&self, mut buffer: Vec<u8>) {
            if buffer.len() != self.buffer_len {
                buffer.resize(self.buffer_len, 0);
            }
            let mut guard = self.buffers.lock().unwrap();
            guard.push(buffer);
        }
    }

    pub struct LibcameraCapture {
        handle: NonNull<LibcameraBridgeOpaque>,
        width: u32,
        height: u32,
        stride: usize,
        bytes_per_pixel: usize,
        pool: Arc<BufferPool>,
        color_format: ColorFormat,
        frame_counter: u64,
    }

    impl LibcameraCapture {
        const LOG_INTERVAL: u64 = 120;

        pub fn new(width: u32, height: u32, fps: u32) -> Result<Self> {
            println!(
                "[bsw][libcamera] new() requested width={} height={} fps={}",
                width, height, fps
            );

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
                eprintln!(
                    "[bsw][libcamera] bridge_create failed: {} (width={} height={} fps={})",
                    msg, width, height, fps
                );
                return Err(opencv_err(msg));
            }

            let stride_usize = stride as usize;
            let bytes_per_pixel = cmp::max(1usize, bpp as usize);
            let buffer_len = stride_usize
                .checked_mul(height as usize)
                .ok_or_else(|| opencv_err("libcamera buffer size overflow"))?;
            let pool = BufferPool::new(buffer_len, 4);
            let color_format = match bytes_per_pixel {
                1 => ColorFormat::Gray,
                3 => ColorFormat::Rgb,
                4 => ColorFormat::Rgba,
                other => {
                    return Err(opencv_err(format!(
                        "Unsupported pixel size during init: {} bytes",
                        other
                    )));
                }
            };

            println!(
                "[bsw][libcamera] init stride={} bytes_per_pixel={} dims={}x{} buffer_len={}",
                stride_usize, bytes_per_pixel, width, height, buffer_len
            );

            Ok(Self {
                handle: unsafe { NonNull::new_unchecked(handle) },
                width,
                height,
                stride: stride_usize,
                bytes_per_pixel,
                pool,
                color_format,
                frame_counter: 0,
            })
        }

        pub fn capture_frame(&mut self) -> Result<Option<CapturedFrame>> {
            let mut out_size = 0usize;
            let mut timestamp_ns = 0u64;
            let mut err_buf = [0 as c_char; ERR_BUF_LEN];
            let mut buffer = self.pool.acquire();

            let rc = unsafe {
                libcamera_bridge_capture(
                    self.handle.as_ptr(),
                    buffer.as_mut_ptr(),
                    buffer.len(),
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
                eprintln!(
                    "[bsw][libcamera] capture error rc={} out_size={} stride={} bpp={}",
                    rc, out_size, self.stride, self.bytes_per_pixel
                );
                return Err(opencv_err(msg));
            }

            if out_size == 0 {
                println!("[bsw][libcamera] capture returned empty frame");
                return Ok(None);
            }

            if out_size < buffer.len() {
                buffer.truncate(out_size);
            }

            self.frame_counter = self.frame_counter.wrapping_add(1);
            if self.frame_counter % Self::LOG_INTERVAL == 0 {
                println!(
                    "[bsw][libcamera] captured frame {}x{} bytes={} ts_ns={}",
                    self.width, self.height, out_size, timestamp_ns
                );
            }

            let recycler: Arc<dyn BufferRecycler> = self.pool.clone();
            let camera_buffer = CameraBuffer::from_vec_with_recycler(buffer, recycler);

            Ok(Some(CapturedFrame::new(
                Arc::new(camera_buffer),
                self.width,
                self.height,
                self.stride,
                self.bytes_per_pixel,
                self.color_format,
            )))
        }

        pub fn dimensions(&self) -> (u32, u32) {
            (self.width, self.height)
        }

        pub fn color_format(&self) -> ColorFormat {
            self.color_format
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
