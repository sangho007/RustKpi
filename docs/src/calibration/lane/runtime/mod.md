# lane/runtime/mod.rs — 런타임 제어 파라미터

- 경로: `src/calibration/lane/runtime/mod.rs`
- 계층: Calibration / Lane Detection / Runtime

## 목적
차선 파이프라인 실행 주기(`process_interval`)를 정의해 프레임 건너뛰기 빈도를 제어합니다. 기본값은 3프레임마다 처리입니다.

