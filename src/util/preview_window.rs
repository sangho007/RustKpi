use crate::rte::rte_dto::ColorFormat;
use opencv::Error;
use opencv::core::StsError;
use sdl2::VideoSubsystem;
use sdl2::pixels::PixelFormatEnum;
use sdl2::render::Canvas;
use sdl2::video::Window;

pub struct SdlPreview {
    canvas: Canvas<Window>,
    frame_width: u32,
    frame_height: u32,
    frame_format: ColorFormat,
    scratch: Vec<u8>,
}

impl SdlPreview {
    pub fn new(
        video: &VideoSubsystem,
        title: &str,
        width: u32,
        height: u32,
        format: ColorFormat,
        position: Option<(i32, i32)>,
    ) -> opencv::Result<Self> {
        let mut builder = video.window(title, width, height);
        let builder = if let Some((x, y)) = position {
            builder.position(x, y)
        } else {
            builder.position_centered()
        };
        let mut window = builder
            .resizable()
            .allow_highdpi()
            .build()
            .map_err(sdl_err)?;

        window.raise();

        let canvas = window
            .into_canvas()
            .accelerated()
            .present_vsync()
            .build()
            .map_err(sdl_err)?;

        Ok(Self {
            canvas,
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
    ) -> opencv::Result<()> {
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

        Ok(())
    }

    pub fn window_id(&self) -> u32 {
        self.canvas.window().id()
    }

    pub fn raise(&mut self) {
        self.canvas.window_mut().raise();
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
