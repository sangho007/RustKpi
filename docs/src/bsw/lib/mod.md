# bsw/lib/mod.rs — BSW 라이브러리 인덱스

- 경로: `src/bsw/lib/mod.rs`
- 계층: BSW / 라이브러리 집합

## 목적
- 센서·액추에이터 계층에서 반복되는 캡처, 프로토콜 변환, 캘리브레이션 도움 함수를 한곳에 모아 재사용성을 높입니다.
- 하드웨어 의존 부분을 분리해 상위 러너블이 간결한 로직만 유지하도록 지원합니다.

## 하위 모듈
- `cam_lib`: 카메라 캡처 추상화(`FrameCapture`, `CapturedFrame`, libcamera 브리지)와 `CameraBuffer` 헬퍼.
- `pwm_lib`: PCA9685 제어에 필요한 캘리브레이션 구조체와 서보/모터 보조 함수.
- `ultrasonic_lib`: 초음파 센서용 `UltrasonicCalibration` 래퍼.
- `imu_proto`: Swift 앱이 전송한 protobuf 텔레메트리를 `DtoImu` 구조체로 변환합니다.
