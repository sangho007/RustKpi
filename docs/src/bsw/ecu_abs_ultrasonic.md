# ecu_abs_ultrasonic.rs — ECU Abstraction (Ultrasonic)

- 경로: `src/bsw/ecu_abs_ultrasonic.rs`
- 계층: BSW / ECU Abstraction (Ultrasonic)

## 목적
HC‑SR04 초음파 센서를 주기적으로 폴링하여 거리(cm)를 `DtoUltraSonicRaw`로 게시합니다.

## 동작 요약
- 100ms 주기로 `HcSr04::measure_distance` 호출.
- 성공 시 `raw_tx.send(Arc<DtoUltraSonicRaw>)`, 범위 초과는 통계 누적, 에러는 로그.
- 1초마다 정상/범위초과/최근거리 요약 로그 출력.

## 의존성
- 내부: `bsw::lib::ultrasonic_lib::{TRIGGER_PIN,ECHO_PIN}`
- 외부: `hc_sr04`

## 데이터 플로우
- 출력: `UltrasonicChannels.raw_tx`
- 소비자 예: `asw::uss_forwardcollision`이 장애물 판정으로 사용

## 에러 처리
- 초기화 실패 시 로그 후 태스크 종료(하드웨어 미가용 대비).
- 측정 에러는 로그만 남기고 루프 지속.

