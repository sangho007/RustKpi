use opencv::core::Point2f;

#[derive(Clone, Copy, Debug)]
pub struct PerspectiveCalibration {
    pub source: [(f32, f32); 4],
    pub destination: [(f32, f32); 4],
}

impl PerspectiveCalibration {
    pub fn source_points(&self) -> Vec<Point2f> {
        self.source
            .iter()
            .map(|&(x, y)| Point2f::new(x, y))
            .collect()
    }

    pub fn destination_points(&self) -> Vec<Point2f> {
        self.destination
            .iter()
            .map(|&(x, y)| Point2f::new(x, y))
            .collect()
    }
}

impl Default for PerspectiveCalibration {
    fn default() -> Self {
        Self {
            source: [
                (200.0, 620.0),
                (540.0, 480.0),
                (740.0, 480.0),
                (1080.0, 620.0),
            ],
            destination: [(200.0, 720.0), (300.0, 0.0), (980.0, 0.0), (1080.0, 720.0)],
        }
    }
}
