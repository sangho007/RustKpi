# preview_runtime.rs — SDL 프리뷰 런타임

- 경로: `src/util/preview_runtime.rs`
- 계층: Util / GUI

## 역할
별도 스레드에서 SDL 기반 프리뷰 창을 실행하고, RTE에서 전달된 카메라/전처리/Bird-eye 프레임을 표시합니다. 종료 이벤트를 메인 태스크로 전달해 안전하게 프로그램을 끝낼 수 있도록 합니다.

## 구성 요소
- `PreviewRuntime`: 프리뷰 스레드 핸들 및 송수신 채널을 묶은 구조체.
- `FramePacket` & `FramePayload`: 프레임 메타데이터와 실제 픽셀 데이터를 캡슐화합니다.
- `PreviewMessage` / `PreviewEvent`: 프리뷰 스레드와 메인 스레드 간 양방향 통신을 위한 enum.
- `spawn_preview_thread`: 스레드를 생성하고 SDL 초기화 실패 시 OpenCV 에러로 변환합니다.

