// asw/vision

use crate::asw::lib::vs_lane_lib::*;
use crate::rte::rte_dto::{DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed};
use crate::rte::rte_main::CameraChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_pre_processing(id: &'static str, camera: CameraChannels) -> opencv::Result<()> {
    let raw_tx = camera.raw_tx.clone();
    let processed_tx = camera.processed_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = raw_tx.subscribe();
        let mut alive_cnt = 0;

        let pipeline = Pipeline::new()?;

        loop {
            // 1. 이벤트 수신 및 데이터 준비
            let cam_raw = match rx.blocking_recv() {
                Ok(cam_dto) => cam_dto, // 처리할 데이터만 추출
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[{}] PreProcess lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            };

            // 2. 실제 연산 처리
            let gray = pipeline.gray_scale(&cam_raw.img)?;
            let blur = pipeline.noise_removal(&gray)?;
            let edges = pipeline.edge_detection(&blur)?;
            let closed = pipeline.morphology_close(&edges)?;

            // 3. 결과 전송
            let preprocessed_dto = Arc::new(DtoCamProcessed::new(Arc::new(closed), 1280, 720, alive_cnt));
            let _ = processed_tx.send(preprocessed_dto);

            alive_cnt += 1;
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Lane pre-processing task join error: {}", e),
        )
    })?;

    join_result
}

pub async fn runnable_get_lane_angle(id: &'static str, camera: CameraChannels) -> opencv::Result<()> {
    let processed_tx = camera.processed_tx.clone();
    let bird_eye_tx = camera.bird_eye_tx.clone();
    let lane_angle_tx = camera.lane_angle_tx.clone();
    let join_result = tokio::task::spawn_blocking(move || -> opencv::Result<()> {
        let mut rx = processed_tx.subscribe();
        let mut alive_cnt: u32 = 0;

        let mut pipeline = Pipeline::new()?;
        loop {
            // 1. 이벤트 수신 및 데이터 준비
            let cam_processed = match rx.blocking_recv() {
                Ok(cam_dto) => cam_dto, // 처리할 데이터만 추출
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[{}] LaneAngle lagged by {}", id, n);
                    continue;
                }
                Err(RecvError::Closed) => break,
            };

            // 2. 실제 연산 처리
            // 3. roi 이미지 추출
            let roi_img = pipeline.roi(&cam_processed.img)?;

            // 4. bird-eye-view 이미지 생성
            let birds_eye_img = pipeline.perspective_transform(&roi_img)?;

            // 5. 슬라이딩 윈도우
            let left_fitx: Vec<f64>;
            let right_fitx: Vec<f64>;
            let left_lane_detected: bool;
            let right_lane_detected: bool;

            // 조건: 이전에 차선을 성공적으로 찾은 기록이 있는가?
            let has_previous_detection = !pipeline.left_a.is_empty() && !pipeline.right_a.is_empty();

            if has_previous_detection {
                // [빠른 추적] 이전 기록이 있으면, 그 주변만 빠르게 탐색합니다.
                let (_img, lfx, rfx, l_det, r_det) = pipeline.search_around_poly(&birds_eye_img, 100)?;

                // 결과 할당
                left_fitx = lfx;
                right_fitx = rfx;
                left_lane_detected = l_det;
                right_lane_detected = r_det;

                // [실패 처리] 만약 빠른 추적에 실패했다면, 다음 프레임에서 전체 탐색을 하도록 상태를 리셋합니다.
                if !(left_lane_detected || right_lane_detected) {
                    pipeline.left_a.clear();
                    pipeline.left_b.clear();
                    pipeline.left_c.clear();
                    pipeline.right_a.clear();
                    pipeline.right_b.clear();
                    pipeline.right_c.clear();
                }
            } else {
                // [전체 탐색] 이전 기록이 없으면, sliding_window로 전체 영역을 탐색합니다.
                let (_img, lfx, rfx, l_det, r_det) =
                    pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;

                // 결과 할당
                left_fitx = lfx;
                right_fitx = rfx;
                left_lane_detected = l_det;
                right_lane_detected = r_det;
            }

            let steering_angle = pipeline.get_angle_on_lane(
                &left_fitx,
                &right_fitx,
                left_lane_detected,
                right_lane_detected,
            );

            // 3. 결과 전송
            let lane_angle_dto = Arc::new(DtoCamLaneAngle::new(steering_angle, alive_cnt));
            let _ = lane_angle_tx.send(lane_angle_dto);

            let birds_eye_view_dto = Arc::new(DtoCamBirdEyeView::new(Arc::new(birds_eye_img), 1280, 720, alive_cnt));
            let _ = bird_eye_tx.send(birds_eye_view_dto);

            alive_cnt += 1;
        }

        Ok(())
    })
    .await
    .map_err(|e| {
        opencv::Error::new(
            opencv::core::StsError,
            format!("Lane angle task join error: {}", e),
        )
    })?;

    join_result
}
