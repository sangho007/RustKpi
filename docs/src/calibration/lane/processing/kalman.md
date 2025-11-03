# lane/processing/kalman.rs — 칼만 필터 설정

- 경로: `src/calibration/lane/processing/kalman.rs`
- 계층: Calibration / Lane Detection / Processing

## 목적
- 차선 각도 추정에 사용되는 칼만 필터 파라미터를 캘리브레이션으로 분리해 환경별 노이즈 특성에 맞출 수 있도록 합니다.

## 제공 항목
- 칼만 필터 활성화 여부, 공정/관측 노이즈, 초기 상태·공분산을 정의해 차선 각도 추정 안정성을 제어합니다.
