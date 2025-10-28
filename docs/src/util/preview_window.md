# preview_window.rs — SDL 프레임 렌더러

- 경로: `src/util/preview_window.rs`
- 계층: Util / GUI

## 목적
SDL 창/캔버스를 감싸 프레임 데이터를 텍스처로 변환하고 화면에 그리거나, 해상도·포맷 변경을 자동으로 처리합니다.

## 특징
- 회색조 입력을 RGB24로 확장해 SDL이 요구하는 형식에 맞춥니다.
- 창 위치/크기 변경, raise 등 사용자 경험 관련 유틸리티를 제공합니다.

