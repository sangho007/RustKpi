use crate::rte::rte_dto::ColorFormat;
use opencv::Error;
use opencv::core::StsError;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::PixelFormatEnum;
use sdl2::render::Canvas;
use sdl2::video::Window;

pub struct SdlPreview {
    _sdl: sdl2::Sdl,
    canvas: Canvas<Window>,
    event_pump: sdl2::EventPump,
    window_title: String,
    frame_width: u32,
    frame_height: u32,
    frame_format: ColorFormat,
    scratch: Vec<u8>,
}

impl SdlPreview {
    pub fn new(
        title: impl Into<String>,
        width: u32,
        height: u32,
        format: ColorFormat,
    ) -> opencv::Result<Self> {
        let sdl = sdl2::init().map_err(sdl_err)?;
        let video = sdl.video().map_err(sdl_err)?;

        let title_str = title.into();
        let window = video
            .window(&title_str, width, height)
            .position_centered()
            .resizable()
            .allow_highdpi()
            .build()
            .map_err(sdl_err)?;

        let canvas = window
            .into_canvas()
            .accelerated()
            .present_vsync()
            .build()
            .map_err(sdl_err)?;

        let event_pump = sdl.event_pump().map_err(sdl_err)?;

        Ok(Self {
            _sdl: sdl,
            canvas,
            event_pump,
            window_title: title_str,
            frame_width: width,
            frame_height: height,
            frame_format: format,
            scratch: Vec::new(),
        })
    }

    pub fn present(
        &mut self,
        width: u32,
        height: u32,
        format: ColorFormat,
        data: &[u8],
        stride: usize,
    ) -> opencv::Result<bool> {
        if width != self.frame_width || height != self.frame_height {
            self.canvas
                .window_mut()
                .set_size(width, height)
                .map_err(sdl_err)?;
            self.frame_width = width;
            self.frame_height = height;
        }

        if format != self.frame_format {
            self.frame_format = format;
            self.scratch.clear();
        }

        let (pixel_format, needs_gray_expand) = texture_config(self.frame_format);

        let (pixels, pitch): (&[u8], usize) = if needs_gray_expand {
            let expected = (width * height) as usize;
            if data.len() != expected {
                return Err(Error::new(
                    StsError,
                    "Unexpected grayscale buffer length for SDL preview",
                ));
            }
            if self.scratch.len() != expected * 3 {
                self.scratch.resize(expected * 3, 0);
            }
            for (i, value) in data.iter().enumerate() {
                let base = i * 3;
                self.scratch[base] = *value;
                self.scratch[base + 1] = *value;
                self.scratch[base + 2] = *value;
            }
            (&self.scratch, (width * 3) as usize)
        } else {
            (data, stride)
        };

        let texture_creator = self.canvas.texture_creator();
        let mut texture = texture_creator
            .create_texture_streaming(pixel_format, width, height)
            .map_err(sdl_err)?;
        texture.update(None, pixels, pitch).map_err(sdl_err)?;

        self.canvas.clear();
        self.canvas.copy(&texture, None, None).map_err(sdl_err)?;
        self.canvas.present();

        Ok(self.process_events())
    }

    fn process_events(&mut self) -> bool {
        for event in self.event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => return true,
                _ => {}
            }
        }
        false
    }

    pub fn title(&self) -> &str {
        &self.window_title
    }
}

fn texture_config(format: ColorFormat) -> (PixelFormatEnum, bool) {
    match format {
        ColorFormat::Bgr => (PixelFormatEnum::BGR24, false),
        ColorFormat::Rgb => (PixelFormatEnum::RGB24, false),
        ColorFormat::Rgba => (PixelFormatEnum::RGBA32, false),
        ColorFormat::Gray => (PixelFormatEnum::RGB24, true),
    }
}

fn sdl_err<E: ToString>(err: E) -> Error {
    Error::new(StsError, err.to_string())
}
