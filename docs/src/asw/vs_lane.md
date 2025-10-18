# vs_lane.rs — 차선 인식 태스크

- 경로: `src/asw/vs_lane.rs`
- 계층: ASW / Vision (Lane)

## 목적
Lane 파이프라인을 두 단계로 분리하여 처리합니다.
1) `runnable_pre_processing`: Raw → Gray/Blur/Edge/Morphology 처리된 프레임 게시(Processed)
2) `runnable_get_lane_angle`: Processed → Bird’s‑eye 변환, 차선 검출, 조향각(LaneAngle) 계산

## 데이터 플로우
- 입력: `CameraChannels.raw_tx`(pre), `processed_tx`(angle)
- 출력: `processed_tx`(pre), `bird_eye_tx`/`lane_angle_tx`(angle)

## 성능/구성
- `PROCESS_INTERVAL = 3` 간격으로 처리(캐시된 프레임 재사용)해 부하 저감
- `LaneTaskConfig::use_kalman` 기본 false (필요 시 `update_angle_kalman` 적용)

## 동시성
- 각 태스크는 `spawn_blocking`으로 CPU 바운드 연산을 워커 스레드에서 수행
- 채널 `Lagged/Closed` 시 로그 처리 후 루프 지속/종료

## 에러 처리
- Join 에러를 OpenCV `Error`로 포장하여 상위로 전파

