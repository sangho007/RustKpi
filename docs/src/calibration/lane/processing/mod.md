# lane/processing/mod.rs — 영상 처리 캘리브레이션

- 경로: `src/calibration/lane/processing/mod.rs`
- 계층: Calibration / Lane Detection / Processing

## 목적
- 차선 영상 처리에 필요한 다단계 캘리브레이션을 구조체 하나로 묶어 일관된 파이프라인을 구성합니다.

## 제공 항목
- `ProcessingCalibration`: 필터링, 모폴로지, 슬라이딩 윈도우, 칼만 필터 하위 설정을 한 구조체에 담습니다.
- 각 서브 캘리브레이션은 기본값을 통해 즉시 사용 가능한 파이프라인을 구성합니다.
