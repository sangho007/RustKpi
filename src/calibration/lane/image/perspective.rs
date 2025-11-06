//! 원근 투시 변환에 필요한 좌표 쌍을 정의한다.

use opencv::core::Point2f;

#[derive(Clone, Copy, Debug)]
/// 원근 변환(IRL)과 버드아이 변환(목표) 좌표.
pub struct PerspectiveCalibration {
    pub source: [(f32, f32); 4],
    pub destination: [(f32, f32); 4],
}

impl PerspectiveCalibration {
    /// 소스/목적지 좌표를 지정해 초기화한다.
    pub fn new(source: [(f32, f32); 4], destination: [(f32, f32); 4]) -> Self {
        Self {
            source,
            destination,
        }
    }

    /// OpenCV가 요구하는 `Point2f` 리스트로 변환한다(원본 좌표).
    pub fn source_points(&self) -> Vec<Point2f> {
        self.source
            .iter()
            .map(|&(x, y)| Point2f::new(x, y))
            .collect()
    }

    /// 목적 좌표를 `Point2f` 리스트로 반환한다.
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
                (0.0, 240.0),
                (160.0, 150.0),
                (480.0, 150.0),
                (640.0, 240.0),
            ],
            destination: [(0.0, 480.0), (0.0, 0.0), (640.0, 0.0), (640.0, 480.0)],
        }
    }
}
