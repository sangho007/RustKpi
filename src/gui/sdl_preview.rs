use crate::rte::rte_dto::{ColorFormat, DtoCamRaw};
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
    frame_width: u32,
    frame_height: u32,
    frame_format: ColorFormat,
    scratch: Vec<u8>,
}

impl SdlPreview {
    pub fn new(width: u32, height: u32, format: ColorFormat) -> opencv::Result<Self> {
        let sdl = sdl2::init().map_err(sdl_err)?;
        let video = sdl.video().map_err(sdl_err)?;

        let window = video
            .window("Raw Preview", width, height)
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
            frame_width: width,
            frame_height: height,
            frame_format: format,
            scratch: Vec::new(),
        })
    }

    pub fn present(&mut self, frame: &DtoCamRaw) -> opencv::Result<bool> {
        if frame.width != self.frame_width || frame.height != self.frame_height {
            self.canvas
                .window_mut()
                .set_size(frame.width, frame.height)
                .map_err(sdl_err)?;
            self.frame_width = frame.width;
            self.frame_height = frame.height;
        }

        if frame.color_format != self.frame_format {
            self.frame_format = frame.color_format;
            self.scratch.clear();
        }

        let (pixel_format, needs_gray_expand) = texture_config(self.frame_format);

        let (data, pitch): (&[u8], usize) = if needs_gray_expand {
            let expected = (frame.width * frame.height) as usize;
            let source = frame.buffer.as_slice();
            if source.len() != expected {
                return Err(Error::new(
                    StsError,
                    "Unexpected grayscale buffer length for SDL preview",
                ));
            }
            if self.scratch.len() != expected * 3 {
                self.scratch.resize(expected * 3, 0);
            }
            for (i, value) in source.iter().enumerate() {
                let base = i * 3;
                let color = *value;
                self.scratch[base] = color;
                self.scratch[base + 1] = color;
                self.scratch[base + 2] = color;
            }
            (&self.scratch, (frame.width * 3) as usize)
        } else {
            (frame.buffer.as_slice(), frame.stride)
        };

        let texture_creator = self.canvas.texture_creator();
        let mut texture = texture_creator
            .create_texture_streaming(pixel_format, frame.width, frame.height)
            .map_err(sdl_err)?;
        texture.update(None, data, pitch).map_err(sdl_err)?;

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
