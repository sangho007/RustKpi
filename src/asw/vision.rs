// asw/vision

use std::sync::Arc;
use opencv::prelude::Mat;
use crate::rte::rte_main::{DebugSender, VfbSender};
use crate::rte::rte_dto::{VfbEvent, DtoCamProcessed, DtoCamLaneAngle, DtoCamBirdEyeView};
use crate::asw::lib::vision_lib::*;
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_pre_processing(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<()>  {
    let mut rx = tx.subscribe();
    let mut alive_cnt = 0;

    let mut pipeline = Pipeline::new()?;
    let mut gray = Mat::default();
    let mut blur = Mat::default();
    let mut edges = Mat::default();

    loop {
        // 1. 이벤트 수신 및 데이터 준비
        let cam_raw = match rx.recv().await {
            Ok(VfbEvent::CamRawData(data)) => data, // 처리할 데이터만 추출
            Err(RecvError::Lagged(n)) => { continue; }
            _ => { continue; } // 관심 없는 이벤트는 무시
        };

        // 2. await 이후, 실제 연산 처리
        // `match` 블록 바깥에서 모든 연산을 수행합니다.
        let mut closed = Mat::default();

        // 1) 그레이 변환
        gray = pipeline.gray_scale(&cam_raw.img)?;

        // 2) 가우시안 블러
        blur = pipeline.noise_removal(&gray)?;

        // 3) 캐니 엣지
        edges = pipeline.edge_detection(&blur)?;

        // 4) 모폴로지 닫힘
        closed = pipeline.morphology_close(&edges)?;

        // 3. 결과 전송
        let cam_preprocessed = DtoCamProcessed::new(Arc::new(closed), 1280, 720, alive_cnt);
        let event = VfbEvent::CamProcessedData(Arc::new(cam_preprocessed));

        let _ = tx.send(event.clone());
        let _ = debug.send(event.clone());

        alive_cnt += 1;
    }
}

pub async fn runnable_get_lane_angle(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<()>  {
    let mut rx = tx.subscribe();
    let mut alive_cnt: u32 = 0;

    let mut pipeline = Pipeline::new()?;
    let mut roi_img = Mat::default();

    loop {
        // 1. 이벤트 수신 및 데이터 준비
        let cam_processed = match rx.recv().await {
            Ok(VfbEvent::CamProcessedData(data)) => data, // 처리할 데이터만 추출
            Err(RecvError::Lagged(n)) => {
                eprintln!("[{}] Error receiving event: {}", id, n);
                continue;
            }
            _ => { continue; } // 관심 없는 이벤트는 무시
        };

        // 2. await 이후, 실제 연산 처리
        let mut birds_eye_img = Mat::default();

        roi_img = pipeline.roi(&cam_processed.img)?;
        birds_eye_img = pipeline.perspective_transform(&roi_img)?;

        let (_sliding_window_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected) =
            pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;

        let steering_angle = pipeline.get_angle_on_lane(
            &left_fitx,
            &right_fitx,
            left_lane_detected,
            right_lane_detected,
        );

        // 3. 결과 전송
        let lane_angle_dto = DtoCamLaneAngle::new(steering_angle, alive_cnt);
        let event = VfbEvent::CamLaneAngleData(Arc::new(lane_angle_dto));
        let _ = tx.send(event.clone());
        let _ = debug.send(event.clone());

        let birds_eye_view_dto = DtoCamBirdEyeView::new(Arc::new(birds_eye_img), 1280, 720, alive_cnt);
        let event2 = VfbEvent::CamCamBirdEyeViewData(Arc::new(birds_eye_view_dto));
        let _ = debug.send(event2.clone());

        alive_cnt += 1;
    }
}