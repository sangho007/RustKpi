# ecu_abs_ultrasonic.rs - ECU Abstraction (Ultrasonic)

- 경로: `src/bsw/ecu_abs_ultrasonic.rs`
- 계층: BSW / ECU Abstraction (Ultrasonic)

## 목적
HC-SR04 초음파 센서를 폴링해 거리(cm)를 측정하고 `DtoUltraSonicRaw`로 브로드캐스트합니다. 기본 주기는 `UltrasonicCalibration::default()`에 정의되어 있으며 장애물 감지 ASW 파이프라인의 입력이 됩니다.

## 실행 흐름
1. `ultrasonic_calibration()`으로 트리거/에코 핀, 샘플 주기, 로그 주기를 읽어옵니다.
2. `HcSr04::new`으로 센서를 초기화하고 실패하면 경고 후 즉시 반환합니다.
3. `tokio::time::interval(calibration.sample_interval)`을 사용해 비동기 주기를 유지하면서 `measure_distance(Unit::Centimeters)`를 호출합니다.
4. 측정이 성공하면 `DtoUltraSonicRaw::new(distance, alive_cnt)`를 생성해 `UltrasonicChannels.raw_tx`에 전송합니다.
5. 범위 초과(`Ok(None)`)와 측정 에러는 각각 카운터 증가 또는 로그만 남기고 루프를 계속 이어갑니다.
6. `calibration.log_interval`마다 정상/범위 초과 횟수와 최근 거리를 요약 로그로 출력합니다.

## 상태 관리
- `alive_cnt`는 샘플 번호를 나타내며 `u32`에서 wrap 합니다.
- `last_distance`는 가장 최근 성공 값을 보관해 주기 로그에 사용됩니다.
- `in_range_samples`와 `out_of_range_samples`는 로그 주기 동안의 통계를 누적합니다.

## 데이터 플로우
- 출력: `UltrasonicChannels.raw_tx` (`broadcast::Sender<Arc<DtoUltraSonicRaw>>`)
- 주요 소비자: `asw::forwardcollision_ultrasonic` 장애물 판정, `main_runtime` 로그 출력

## 장애 대응
- 초기화 실패는 하드웨어 미가용 상황으로 간주하고 태스크를 종료합니다.
- 개별 측정 실패는 루프를 유지해 일시적 노이즈나 반사 문제를 허용합니다.
