# util/mod.rs — 유틸리티 모듈 루트

- 경로: `src/util/mod.rs`
- 계층: Util

## 목적
- 프리뷰 GUI 및 SDL 초기화 관련 유틸리티 모듈을 한 곳에서 노출해 상위 레이어가 명확히 가져올 수 있게 합니다.

## 하위 모듈
- `preview_runtime`: SDL 프리뷰 루프 실행.
- `preview_window`: 창 생성과 이벤트 관리.
- `sdl_env`: SDL 컨텍스트 초기화 헬퍼.
