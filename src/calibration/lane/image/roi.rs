use opencv::core::Point;

#[derive(Clone, Copy, Debug)]
pub struct RoiCalibration {
    pub vertices: [(i32, i32); 4],
}

impl RoiCalibration {
    pub fn new(vertices: [(i32, i32); 4]) -> Self {
        Self { vertices }
    }

    pub fn to_points(&self) -> Vec<Point> {
        self.vertices
            .iter()
            .map(|&(x, y)| Point::new(x, y))
            .collect()
    }
}

impl Default for RoiCalibration {
    fn default() -> Self {
        Self {
            vertices: [(100, 413), (270, 320), (370, 320), (540, 413)],
        }
    }
}
