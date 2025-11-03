# camera_lib.rs — 카메라 버퍼 헬퍼

- 경로: `src/rte/lib/camera_lib.rs`
- 계층: RTE / Library

## 목적
- RTE 단계에서 카메라 버퍼를 안전하게 전달하고 색상 포맷을 일관되게 변환하기 위한 헬퍼를 제공합니다.
- 복사 없이 OpenCV `Mat` 뷰를 구성해 성능을 유지합니다.

## 주요 구성
- `CameraBuffer`: 캡처한 프레임 데이터를 소유하고 선택적으로 재활용 콜백(`BufferRecycler`)을 호출합니다.
- `ColorFormat`: 지원하는 입력 색상 포맷(BGR, RGB, RGBA, Gray)을 정의합니다.
- `mat_from_buffer`: 버퍼 포인터를 OpenCV `Mat`으로 매핑해 복사 없이 처리합니다.
- `ensure_bgr`: 다양한 포맷을 BGR `Mat`으로 변환해 후속 파이프라인을 단순화합니다.
