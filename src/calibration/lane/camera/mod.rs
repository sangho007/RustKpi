//! 차선 검출에 사용되는 카메라 설정을 정의한다.

#[derive(Clone, Copy, Debug)]
/// 영상 입력 디바이스의 해상도·프레임레이트 및 소스 경로 설정.
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
    /// 목표 FPS를 기반으로 한 프레임 간격을 계산한다.
    pub fn frame_interval(&self) -> std::time::Duration {
        if self.target_fps == 0 {
            return std::time::Duration::from_secs(0);
        }
        let nanos_per_frame = 1_000_000_000u64 / self.target_fps as u64;
        std::time::Duration::from_nanos(nanos_per_frame)
    }

    /// 가로 해상도를 `u32`로 반환한다.
    pub fn width_u32(&self) -> u32 {
        self.width as u32
    }

    /// 세로 해상도를 `u32`로 반환한다.
    pub fn height_u32(&self) -> u32 {
        self.height as u32
    }
}
