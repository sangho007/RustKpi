# vs_lane_lib.rs - 차선 인식 파이프라인 라이브러리

- 경로: `src/asw/lib/vs_lane_lib.rs`
- 계층: ASW / 라이브러리 (Vision)

## 핵심 타입
- `Pipeline`: 차선 인식에 필요한 ROI, 투시 변환 행렬, 슬라이딩 윈도우/칼만 필터 상태, 최근 차선 계수 이력 등을 보관합니다. `LaneCalibration::preset`에서 파생된 파라미터로 초기화됩니다.
- `LaneTaskConfig { use_kalman: bool }`: 런타임에서 칼만 필터 사용 여부를 전달하기 위한 간단한 설정 구조체.

## 생성
- `Pipeline::new_with_settings(use_kalman)`:
  - `LaneCalibration::preset(LaneCalibrationPreset::Vga640x480)`을 불러 카메라 해상도, ROI 다각형, 투시 변환 행렬, 필터링/모폴로지/슬라이딩 윈도우 설정을 복사합니다.
  - 칼만 필터 파라미터(초기 추정, 공분산, 노이즈)와 슬라이딩 윈도우 디스플레이/탐색 폭을 세팅합니다.
- `Pipeline::new()`는 칼만 비활성화 기본 설정을 제공합니다.

## 주요 전처리 함수
- `gray_scale`, `noise_removal`, `edge_detection`, `morphology_close`: OpenCV UMat을 사용해 GPU/CPU 가속에 유연하게 대응합니다.
- `mat_to_umat` / `umat_to_mat`: Mat과 UMat 간 변환.
- `roi`: ROI 다각형 내부만 남기고 외부를 마스킹합니다.
- `perspective_transform` / `inv_perspective_transform`: Bird's-eye 변환과 역변환을 수행합니다.

## 차선 탐색 및 각도 계산
- `sliding_window(binary, window_count, margin, minpix, draw_debug)`:
  - 히스토그램 기반 시작점을 찾아 차선 픽셀을 누적하고 곡선을 피팅합니다.
  - 충분한 포인트를 찾지 못하면 내부 상태를 리셋합니다.
- `search_around_poly(binary, search_margin)`: 이전 프레임 계수를 바탕으로 빠른 탐색 경로를 제공합니다.
- `get_angle_on_lane`: 검출된 좌/우 차선 곡선에서 차선 중앙을 추정하고 조향각(deg)을 계산합니다.
- `update_angle_kalman`: `LaneTaskConfig::use_kalman`이 true일 때 1차원 칼만 필터로 각도를 평활화합니다.

## 시각화 및 도우미
- `display_heading_line`, `hconcat_2`: 디버깅용 합성 이미지 제작.
- `polyfit_1d`, `polyfit_2d`, `mean_of_last_10`: 곡선 피팅과 계수 평활화에 사용되는 수학 유틸리티.
- `get_nonzero_points_by_row`: Rayon 병렬 처리를 이용해 이진 영상에서 픽셀 좌표를 추출합니다.

## 튜닝 지점
- `LaneCalibration` 하위 모듈에서 ROI, 투시 행렬, 필터 파라미터, 슬라이딩 윈도우 폭, 프로세스/측정 노이즈 등을 수정하면 전체 파이프라인이 새 설정을 따릅니다.
- `LaneRuntimeCalibration.process_interval`을 통해 프레임간 처리 간격을 조절해 성능과 지연을 맞출 수 있습니다.
