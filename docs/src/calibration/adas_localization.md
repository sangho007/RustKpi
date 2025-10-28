# adas_localization.rs — Localization 프리셋

- 경로: `src/calibration/adas_localization.rs`
- 계층: Calibration / ADAS Localization

## 제공 항목
- `LocalizationMapId`: 사용 가능한 지도 자산(`map_data_*.json`)을 식별합니다.
- `LocalizationMapPreset`: 지도별 출발/도착 프리셋 묶음을 정의합니다.
- `LocalizationScenarioSelection` & `LOCALIZATION_ACTIVE_SCENARIO`: 실험에 사용할 기본 맵/출발/도착 조합을 지정합니다.

## 활용 포인트
- 지도 JSON 경로는 라이브러리(`adas_localization_lib`)에서 바로 로드할 수 있도록 문자열 상수로 관리됩니다.
- 프리셋을 수정하면 로컬라이제이션 러너블이 다른 맵·시나리오로 쉽게 전환됩니다.

