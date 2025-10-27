//! SDL 초기화/자원 관리를 간단히 처리하기 위한 래퍼.

use opencv::Error;
use opencv::core::StsError;

/// SDL 컨텍스트와 비디오/이벤트 서브시스템 핸들.
pub struct SdlEnv {
    _context: sdl2::Sdl,
    pub video: sdl2::VideoSubsystem,
    pub event_pump: sdl2::EventPump,
}

impl SdlEnv {
    /// SDL을 초기화하고 필수 서브시스템을 구성한다.
    pub fn new() -> opencv::Result<Self> {
        let context = sdl2::init().map_err(sdl_to_cv_err)?;
        let video = context.video().map_err(sdl_to_cv_err)?;
        let event_pump = context.event_pump().map_err(sdl_to_cv_err)?;
        Ok(Self {
            _context: context,
            video,
            event_pump,
        })
    }
}

/// SDL 오류를 OpenCV 오류로 변환해 호출자에게 전달한다.
pub fn sdl_to_cv_err<E: ToString>(err: E) -> Error {
    Error::new(StsError, err.to_string())
}
