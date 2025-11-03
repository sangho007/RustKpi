# calibration/mod.rs — 시스템 캘리브레이션 모음

- 경로: `src/calibration/mod.rs`
- 계층: Calibration / Root

## 목적
- ADAS 제어, Localization, 통신, 차선·신호등·초음파 등을 포함한 모든 캘리브레이션 서브모듈을 `pub use`로 재노출합니다.
- 소비 측 코드가 단일 모듈에서 필요한 캘리브레이션 구조체를 가져올 수 있도록 집약합니다.

