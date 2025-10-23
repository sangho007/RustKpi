#[derive(Clone, Copy, Debug)]
pub struct CameraCalibration {
    pub width: i32,
    pub height: i32,
    pub target_fps: u32,
    pub capture_queue_depth: usize,
    pub use_libcamera: bool,
    pub sample_video_preferred: &'static str,
    pub sample_video_fallback: &'static str,
}

impl Default for CameraCalibration {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            target_fps: 30,
            capture_queue_depth: 3,
            use_libcamera: true,
            sample_video_preferred: "./video/challenge_640x480.mp4",
            sample_video_fallback: "./video/challenge.mp4",
        }
    }
}

impl CameraCalibration {
    pub fn frame_interval(&self) -> std::time::Duration {
        if self.target_fps == 0 {
            return std::time::Duration::from_secs(0);
        }
        let nanos_per_frame = 1_000_000_000u64 / self.target_fps as u64;
        std::time::Duration::from_nanos(nanos_per_frame)
    }

    pub fn width_u32(&self) -> u32 {
        self.width as u32
    }

    pub fn height_u32(&self) -> u32 {
        self.height as u32
    }
}
