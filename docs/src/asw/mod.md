# asw/mod.rs — ASW 모듈 인덱스

- 경로: `src/asw/mod.rs`
- 계층: ASW / 상위 모듈

## 목적
- Vision·Localization·경로 계획·제어 등 ADAS 애플리케이션 소프트웨어 태스크와 라이브러리를 한 모듈 트리에서 노출합니다.
- 상위 `main.rs`가 필요한 러너블을 명시적으로 가져오고 테스트 시 단일 엔트리포인트로 사용할 수 있도록 정리합니다.

## 하위 모듈
- `vs_lane`, `vs_trafficlight`: 카메라 기반 차선/신호 인식 러너블.
- `forwardcollision_ultrasonic`: 초음파 장애물 감지.
- `adas_cod`: 횡방향·종방향 제어 루프.
- `adas_localization`: IMU 기반 로컬라이제이션과 도착 판정.
- `adas_path_local`, `adas_path_global`: 경로 계획 파이프라인.
- `lib`: 공통 유틸리티와 캘리브레이션 래퍼.
