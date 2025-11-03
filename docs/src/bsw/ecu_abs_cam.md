# ecu_abs_cam.rs — ECU Abstraction (Camera)

- 경로: `src/bsw/ecu_abs_cam.rs`
- 계층: BSW / ECU Abstraction (Camera)

## 목적
카메라(또는 샘플 비디오 파일)에서 프레임을 캡처해 `DtoCamRaw`로 브로드캐스트합니다. 캡처 스레드와 RTE 송신 루프를 분리해 I/O 지연과 영상 처리 파이프라인을 완충합니다.

## 실행 흐름
1. `CameraCalibration::default()`를 읽어 해상도, 목표 FPS, 큐 크기, 샘플 비디오 경로를 정합니다.
2. 캡처 스레드를 `std::thread::Builder::spawn`으로 띄우고, `camera_capture_loop`에서 `FrameCapture` 구현체(libcamera or OpenCV)를 지속적으로 재초기화·폴링합니다.
3. 캡처 스레드는 `fit_frame_to_target`으로 입력 영상을 ROI 크롭+리사이즈해 보정 해상도로 맞춘 뒤 `CapturedFrame`을 MPSC 채널로 전송합니다.
4. async 컨슈머는 채널에서 프레임을 받으면 `DtoCamRaw::new`로 래핑합니다. DTO에는 `CameraBuffer`(stride, bytes_per_pixel, `ColorFormat`)와 프레임 카운터(alive_cnt)가 포함됩니다.
5. 생성된 DTO를 `CameraChannels.raw_tx` broadcast 채널에 전송합니다.

## 캡처 백엔드 선택
- `camera_calibration.use_libcamera`가 true면 `libcamera_capture::LibcameraCapture`로 하드웨어 카메라를 구동합니다.
- false일 때는 샘플 비디오 경로(`./video/challenge_640x480.mp4` 선호, 없으면 fallback)를 OpenCV `VideoCapture`로 열고, 해상도 정보를 캘리브레이션 값으로 덮어씁니다.

## 타이밍 및 백프레셔
- `CameraCalibration::frame_interval()`을 사용해 목표 FPS에 맞춰 프레임을 일정하게 발행합니다.
- 캡처 스레드는 백엔드 오류나 EOF 발생 시 로그를 남기고 500ms 대기 후 재시도합니다. MPSC 채널이 닫히면 곧바로 종료합니다.
- async 측은 `alive_cnt`를 `wrapping_add`로 갱신해 오버플로를 안전하게 처리하며, broadcast 수신자가 없을 때도 drop 없이 계속 송신합니다.

## 데이터 플로우
- 출력: `CameraChannels.raw_tx` (`broadcast::Sender<Arc<DtoCamRaw>>`)
- 소비자: Lane/TrafficLight ASW 태스크, GUI 프리뷰(`main_runtime`)

## 장애 대응
- 스레드 스폰 실패, join 오류, libcamera 초기화 실패 등은 OpenCV `Error`로 변환해 상위 레이어에 보고합니다.
- 백엔드 초기화/프레임 읽기 실패는 경고 로그 후 루프를 유지하여 일시적 센서 장애에 대응합니다.
