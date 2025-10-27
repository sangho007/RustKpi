# bsw/lib/mod.rs - BSW 라이브러리 인덱스

- 경로: `src/bsw/lib/mod.rs`
- 포함 모듈: `cam_lib`, `pwm_lib`, `ultrasonic_lib`, `imu_proto`

센서·액추에이터 공통 유틸리티를 제공합니다.
- `cam_lib`: 카메라 캡처 추상화(`FrameCapture`, `CapturedFrame`, libcamera 브리지)와 `CameraBuffer` 헬퍼.
- `pwm_lib`: PCA9685 제어에 필요한 캘리브레이션 구조체와 서보/모터 보조 함수.
- `ultrasonic_lib`: 초음파 센서용 `UltrasonicCalibration` 래퍼.
- `imu_proto`: Swift 앱이 전송한 protobuf 텔레메트리를 `DtoImu` 구조체로 변환합니다.
