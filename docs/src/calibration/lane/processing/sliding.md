# lane/processing/sliding.rs — 슬라이딩 윈도우 설정

- 경로: `src/calibration/lane/processing/sliding.rs`
- 계층: Calibration / Lane Detection / Processing

## 목적
- 슬라이딩 윈도우 탐색의 민감도와 안정성을 캘리브레이션으로 조정해 다양한 조명·해상도 환경에 대응합니다.

## 제공 항목
- 윈도우 개수, 검색 폭, 최소 픽셀 수, 디버그 표시 여부 등을 설정해 히스토그램 기반 탐색 방식을 제어합니다.
