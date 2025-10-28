# adas_localization.rs — 지도 기반 로컬라이제이션 러너블

- 경로: `src/asw/adas_localization.rs`
- 계층: ASW / ADAS Localization

## 목적
IMU로부터 수신한 자세·위치 텔레메트리를 `adas_localization_lib`의 헬퍼와 결합해 현재 차량의 XY 좌표와 yaw 를 추정하고, `DtoLocalizationState`를 RTE 채널로 브로드캐스트합니다.

## 주요 흐름
- `LOCALIZATION_ACTIVE_SCENARIO`에서 선택된 지도 JSON과 출발 지점을 로드해 기본 좌표계를 결정합니다.
- IMU broadcast 채널을 구독하여 최신 샘플만 남기고 나머지는 버려 지연을 최소화합니다.
- 각 샘플마다 `process_imu_sample` 호출로 누적 변위를 계산하고 yaw 축을 추정한 뒤 localization 채널에 전파합니다.

## 오류 처리
- 지도 파일 로딩 실패나 잘못된 시작 waypoint는 즉시 로그를 남기고 실행을 중단합니다.
- IMU 채널 지연(`Lagged`) 및 종료(`Closed`) 상황을 감지해 운영자에게 알리고, 종료 시 러너블을 정리합니다.

