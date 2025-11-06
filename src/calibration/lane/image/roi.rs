//! 차선 검출에 사용할 관심 영역(ROI)을 정의한다.

use opencv::core::Point;

#[derive(Clone, Copy, Debug)]
/// 영상 내 관심 영역 꼭짓점 좌표.
pub struct RoiCalibration {
    pub vertices: [(i32, i32); 4],
}

impl RoiCalibration {
    /// 새 꼭짓점 배열로 초기화한다.
    pub fn new(vertices: [(i32, i32); 4]) -> Self {
        Self { vertices }
    }

    /// OpenCV `Point` 벡터로 변환해 사용한다.
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
            vertices:[
            (0, 240),
            (175, 100),
            (480, 100),
            (640, 240),
            ],
        }
    }
}
