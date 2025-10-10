// Asw/Vision.rs

use std::sync::Arc;
use opencv::prelude::Mat;
use crate::Rte::Rte_Main::{DebugSender, VfbSender};
use crate::Rte::Rte_Dto::{VfbEvent, Dto_CamProcessed, Dto_CamLaneAngle, Dto_CamBirdEyeView};
use crate::Asw::Lib::Vision_Lib::*;
use tokio::sync::broadcast::error::RecvError;

pub async fn Runnable_PreProcessing(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<()>  {
    let mut rx = tx.subscribe();
    let mut alive_cnt = 0;

    let mut pipeline = Pipeline::new()?;
    let mut gray = Mat::default();
    let mut blur = Mat::default();
    let mut edges = Mat::default();
    let mut cam_preprocessed;
    let mut event;

    loop {
        let mut closed = Mat::default(); // 소유권 이슈

        match rx.recv().await {
            Ok(VfbEvent::CamRawData(cam_raw)) =>{
                // 1) 그레이 변환
                pipeline.gray_scale(&cam_raw.img, &mut gray);

                // 2) 가우시안 블러
                pipeline.noise_removal(&gray, &mut blur);

                // 3) 캐니 엣지
                pipeline.edge_detection(&blur, &mut edges);

                // 4) 모폴로지 닫힘
                pipeline.morphology_close(&edges, &mut closed);

                cam_preprocessed = Dto_CamProcessed::new(Arc::new(closed), 1280, 720,  alive_cnt);
                event = VfbEvent::CamProcessedData(Arc::new(cam_preprocessed));

                let _ = tx.send(event.clone());
                let _ = debug.send(event.clone());


                alive_cnt += 1;

            }
            Err(RecvError::Lagged(n)) => {continue;}
            _ => {}
        }
    }
    // Ok(())
}

// --- Completed Function ---
pub async fn Runnable_GetLaneAngle(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<()>  {
    let mut rx = tx.subscribe();
    let mut alive_cnt: u32 = 0;

    let mut pipeline = Pipeline::new()?;
    let mut roi_img = Mat::default();
    let mut _sliding_window_img = Mat::default();
    let mut left_fitx;
    let mut right_fitx;
    let mut left_lane_detected;
    let mut right_lane_detected;
    let mut steering_angle;
    let mut lane_angle_dto;
    let mut event;
    let mut birds_eye_view_dto;


    loop {
        let mut birds_eye_img = Mat::default(); // 소유권 이슈

        match rx.recv().await {
            Ok(VfbEvent::CamProcessedData(cam_processed)) => {
                pipeline.roi(&cam_processed.img, &mut roi_img);

                pipeline.perspective_transform(&roi_img, &mut birds_eye_img);

                (_sliding_window_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected) =
                    pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;

                steering_angle = pipeline.get_angle_on_lane(
                    &left_fitx,
                    &right_fitx,
                    left_lane_detected,
                    right_lane_detected,
                );

                lane_angle_dto = Dto_CamLaneAngle::new(steering_angle, alive_cnt);
                event = VfbEvent::CamLaneAngleData(Arc::new(lane_angle_dto));
                let _ = tx.send(event.clone());
                let _ = debug.send(event.clone());

                birds_eye_view_dto = Dto_CamBirdEyeView::new(Arc::new(birds_eye_img), 1280, 720, alive_cnt);
                let event2 = VfbEvent::CamCamBirdEyeViewData(Arc::new(birds_eye_view_dto));
                let _ = debug.send(event2.clone());

                alive_cnt += 1;
            }
            Err(RecvError::Lagged(n)) => {
                eprintln!("[{}] Error receiving event: {}", id, n);
                continue;
            }
            _ => {
                // Ignore other event types.
            }
        }
    }
    // Ok(())
}