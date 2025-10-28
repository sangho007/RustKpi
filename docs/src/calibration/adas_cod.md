# adas_cod.rs — ADAS 제어 캘리브레이션

- 경로: `src/calibration/adas_cod.rs`
- 계층: Calibration / ADAS Control

## 주요 구조체
- `AdasLateralCalibration`: 차선 각도 기반 서보 제어에 필요한 게인, 각도 범위, 레이트 리밋, 채널 인덱스를 제공합니다.
- `AdasLongitudinalCalibration`: 초음파·신호등 상태로 속도를 결정하는 제어 루프 주기, 순항/감속 속도, 감속·정지 거리, 로그 주기를 정의합니다.

## 기본값 특징
- 측면 제어는 중립 90도, ±90도 범위를 기준으로 비례 제어를 수행하며 한 루프당 최대 10도까지 각도 변화를 허용합니다.
- 종방향 제어는 100ms 주기를 기본으로 하고, 60% 순항 속도 / 25% 감속 속도 / 35cm 정지 거리를 사용합니다.

