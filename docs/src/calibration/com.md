# com.rs — 통신 캘리브레이션

- 경로: `src/calibration/com.rs`
- 계층: Calibration / Communication

## 목적
BSW 통신 게이트웨이가 TCP 소켓을 통해 패킷을 주고받을 때 사용할 호스트, 포트, 최대 페이로드 크기를 정의합니다. 기본값은 로컬 루프백(127.0.0.1:4820)과 512KiB 패킷 제한입니다.

