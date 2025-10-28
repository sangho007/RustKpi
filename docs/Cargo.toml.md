# Cargo.toml — 크레이트 매니페스트

- 경로: `Cargo.toml`
- 계층: 루트 설정

## 역할
- 패키지 메타데이터(`name`, `version`, `edition`)와 런타임/빌드 의존성을 정의합니다.
- OpenCV, tokio, sdl2, prost, serde 등 ADAS 파이프라인에서 사용하는 주요 라이브러리를 명시합니다.
- 릴리즈 프로파일을 조정해 `lto=fat`, `panic=abort`, `target-cpu=native`로 성능을 극대화합니다.

