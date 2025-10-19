# ecu_abs_cam.rs — ECU Abstraction (Camera)

- 경로: `src/bsw/ecu_abs_cam.rs`
- 계층: BSW / ECU Abstraction (Camera)

## 목적
카메라(또는 비디오 파일) 프레임을 주기적으로 캡처하여 `DtoCamRaw`로 RTE 카메라 RAW 채널에 게시합니다. 하드웨어/파일 소스를 공통 인터페이스(`FrameCapture`)로 추상화합니다.

## 동작 요약
- `spawn_blocking` 내부 루프에서 프레임 캡처 → 버퍼를 `CameraBuffer`로 포장 → `DtoCamRaw` 구성 → `raw_tx.send`.
- 프레임 간 간격: 약 33ms(대략 30FPS 목표).
- 소스 전환: `cammode`가 true이면 libcamera 브리지를 사용하고, 아니면 `./video/challenge.mp4`를 읽음.

## 의존성
- 내부: `bsw::lib::cam_lib` (`FrameCapture` Trait, `libcamera_capture::LibcameraCapture`)
- 외부: OpenCV `videoio`, libcamera 브리지

## 데이터 플로우
- 출력: `CameraChannels.raw_tx`에 `Arc<DtoCamRaw>` 송신

## 에러 처리
- 캡처 실패/EOF 시 로그 후 루프 종료.
- Join 에러를 OpenCV `Error`로 매핑해 상위로 전달.

## 구성/주의
- 현재 `DtoCamRaw`의 `width/height`는 1280x720으로 고정되어 있어, 실제 입력 해상도와 불일치할 수 있습니다. 파이프라인 가정에 맞춰 정합 필요.
