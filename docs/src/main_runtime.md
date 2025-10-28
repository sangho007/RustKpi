# main_runtime.rs — GUI 연동 메인 루프

- 경로: `src/main_runtime.rs`
- 계층: Application / Runtime

## 역할
- RTE 채널에서 카메라·초음파·IMU 데이터를 비동기 구독하고, 필요 시 SDL 프리뷰 GUI로 중계합니다.
- Ctrl-C 신호를 감시하며, 채널 종료나 GUI 종료 이벤트를 감지하면 루프를 정리합니다.

## 주요 처리
- `select!` 루프에서 최신 프레임만 유지하도록 `try_recv`로 큐를 비웁니다.
- 초음파·IMU 데이터는 로그로 출력해 실시간 상태를 확인합니다.
- `DEBUG_ON` 플래그가 true일 때만 GUI 스레드를 띄워 리소스 사용을 제어합니다.

