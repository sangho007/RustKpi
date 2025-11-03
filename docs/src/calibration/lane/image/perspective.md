# lane/image/perspective.rs — 투시 변환 좌표

- 경로: `src/calibration/lane/image/perspective.rs`
- 계층: Calibration / Lane Detection / Image

## 목적
- 차선 전처리 단계가 사용할 투시 변환 좌표를 중앙에서 정의해 ROI와 일관된 버드아이 뷰를 확보합니다.

## 제공 항목
- `PerspectiveCalibration`: 원근 변환의 소스/목적지 4점 좌표를 보관하고 OpenCV `Point2f` 목록으로 변환하는 헬퍼를 제공합니다.
- 기본값은 640x480 VGA 환경의 차선 영역을 버드아이 뷰로 투영하도록 세팅되어 있습니다.
