# preview_runtime.rs — SDL 프리뷰 런타임

- 경로: `src/util/preview_runtime.rs`
- 계층: Util / GUI

## 목적
별도 스레드에서 SDL 기반 프리뷰 창을 실행하고, RTE에서 전달된 카메라/전처리/Bird-eye/경로 프레임을 표시합니다. 종료 이벤트를 메인 태스크로 전달해 안전하게 프로그램을 끝낼 수 있도록 합니다.

## 구성 요소
- `PreviewRuntime`: 프리뷰 스레드 핸들 및 송수신 채널을 묶은 구조체.
- `FramePacket` & `FramePayload`: 프레임 메타데이터와 실제 픽셀 데이터를 캡슐화합니다.
- `PreviewMessage` / `PreviewEvent`: 프리뷰 스레드와 메인 스레드 간 양방향 통신을 위한 enum. `PreviewMessage::Path`가 추가되어 경로 시각화 프레임을 주고받습니다.
- `spawn_preview_thread`: 스레드를 생성하고 SDL 초기화 실패 시 OpenCV 에러로 변환합니다.

## 프리뷰 창과 단축키
- **Raw View (`R`)**: 카메라 RAW 프레임
- **Processed View (`P`)**: 전처리 Gray 프레임
- **Bird's Eye View (`B`)**: 투시 변환 및 슬라이딩 윈도우 표시
- **Path View (`M`)**: 전역 경로(파랑), 로컬 경로(초록), 현재 위치·헤딩(빨강)을 640×640 캔버스에 200ms 간격으로 갱신

창은 개별적으로 토글하거나 닫을 수 있으며, ESC/창 닫기로 전체 프리뷰를 종료합니다.
