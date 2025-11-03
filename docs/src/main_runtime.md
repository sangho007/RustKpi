# main_runtime.rs — GUI 연동 메인 루프

- 경로: `src/main_runtime.rs`
- 계층: Application / Runtime

## 목적
- RTE 채널에서 카메라·초음파·경로·로컬라이제이션 등 데이터를 비동기 구독하고, 필요 시 SDL 프리뷰 GUI로 중계합니다.
- Ctrl-C 신호를 감시하며, 채널 종료나 GUI 종료 이벤트를 감지하면 루프를 정리합니다.

## 주요 처리
- `select!` 루프에서 최신 프레임만 유지하도록 `try_recv`로 큐를 비웁니다.
- 초음파·IMU·로컬라이제이션 데이터는 로그로 출력해 실시간 상태를 확인합니다.
- `DEBUG_ON` 플래그가 true일 때 SDL 프리뷰 스레드가 활성화되며, 다음 창을 표시합니다:
  - `Raw View`: 카메라 입력 스트림
  - `Processed View`: 차선 추출 파이프라인 결과
  - `Bird's Eye View`: 투시 변환 후 차선 탐색 결과
  - `Path View`: 전역 경로(파랑), 로컬 경로(초록), 현재 위치/헤딩(빨강)을 하나의 캔버스에 중첩 시각화. 주기는 `PATH_PREVIEW_INTERVAL`(기본 200ms).
- 키보드 단축키: `R`(Raw), `P`(Processed), `B`(Bird), `M`(Path)로 각 창 토글. ESC 또는 창 닫기로 종료.

## 튜닝/디버깅 시 주의점
- 경로 시각화는 `DtoAdasGlobalPath`, `DtoAdasLocalPath`, `DtoLocalizationState`가 동시에 존재할 때만 갱신됩니다. 값이 없으면 창이 유지되지 않습니다.
- Path View는 경로 좌표의 min/max를 계산해 640×640 캔버스에 자동 스케일링합니다. 극단적으로 좁은 범위일 경우에도 주행 방향을 식별할 수 있도록 약간의 패딩을 추가합니다.
- 성능 부담을 줄이기 위해 Path View는 최신 프레임만 그리며, 생성 빈도는 `PATH_PREVIEW_INTERVAL`로 제한됩니다. 필요 시 캔버스 크기나 주기를 `src/main_runtime.rs`에서 조절하세요.
