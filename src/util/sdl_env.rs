use opencv::core::StsError;
use opencv::Error;

pub struct SdlEnv {
    _context: sdl2::Sdl,
    pub video: sdl2::VideoSubsystem,
    pub event_pump: sdl2::EventPump,
}

impl SdlEnv {
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

pub fn sdl_to_cv_err<E: ToString>(err: E) -> Error {
    Error::new(StsError, err.to_string())
}
