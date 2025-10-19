use opencv::core::Point2f;

#[derive(Clone, Copy, Debug)]
pub struct PerspectiveCalibration {
    pub source: [(f32, f32); 4],
    pub destination: [(f32, f32); 4],
}

impl PerspectiveCalibration {
    pub fn new(source: [(f32, f32); 4], destination: [(f32, f32); 4]) -> Self {
        Self {
            source,
            destination,
        }
    }

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
                (100.0, 413.33334),
                (270.0, 320.0),
                (370.0, 320.0),
                (540.0, 413.33334),
            ],
            destination: [(100.0, 480.0), (150.0, 0.0), (490.0, 0.0), (540.0, 480.0)],
        }
    }
}
