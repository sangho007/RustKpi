// asw/vision

use crate::asw::lib::vs_lane_lib::*;
use crate::rte::rte_dto::{DtoCamBirdEyeView, DtoCamLaneAngle, DtoCamProcessed, VfbEvent};
use crate::rte::rte_main::{DebugSender, VfbSender};
use opencv::prelude::Mat;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

pub async fn runnable_pre_processing(id: &'static str, tx: VfbSender, debug: DebugSender) -> opencv::Result<()>  {
    let mut rx = tx.subscribe();
    let mut alive_cnt = 0;

    let mut pipeline = Pipeline::new()?;
    let mut gray = Mat::default();
    let mut blur = Mat::default();
    let mut edges = Mat::default();
    let mut preprocessed_dto;
    let mut event;

    loop {
        // 1. 이벤트 수신 및 데이터 준비
        let cam_raw = match rx.recv().await {
            Ok(VfbEvent::CamRawEvent(cam_dto)) => cam_dto, // 처리할 데이터만 추출
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
        preprocessed_dto = DtoCamProcessed::new(Arc::new(closed), 1280, 720, alive_cnt);
        event = VfbEvent::CamProcessedEvent(Arc::new(preprocessed_dto));

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
    let mut lane_angle_dto;
    let mut birds_eye_view_dto;
    let mut event1;
    let mut event2;

    loop {
        // 1. 이벤트 수신 및 데이터 준비
        let cam_processed = match rx.recv().await {
            Ok(VfbEvent::CamProcessedEvent(cam_dto)) => cam_dto, // 처리할 데이터만 추출
            Err(RecvError::Lagged(n)) => {
                eprintln!("[{}] Error receiving event: {}", id, n);
                continue;
            }
            _ => { continue; } // 관심 없는 이벤트는 무시
        };

        // 2. await 이후, 실제 연산 처리
        let mut birds_eye_img = Mat::default();

        // 3. roi 이미지 추출
        roi_img = pipeline.roi(&cam_processed.img)?;

        // 4. bird-eye-view 이미지 생성
        birds_eye_img = pipeline.perspective_transform(&roi_img)?;

        // 5. 슬라이딩 윈도우
        let sliding_window_img: Mat;
        let left_fitx: Vec<f64>;
        let right_fitx: Vec<f64>;
        let left_lane_detected: bool;
        let right_lane_detected: bool;

        // 조건: 이전에 차선을 성공적으로 찾은 기록이 있는가?
        let has_previous_detection = !pipeline.left_a.is_empty() && !pipeline.right_a.is_empty();

        if has_previous_detection {
            // [빠른 추적] 이전 기록이 있으면, 그 주변만 빠르게 탐색합니다.
            let (img, lfx, rfx, l_det, r_det) = pipeline.search_around_poly(&birds_eye_img, 100)?;
            // println!(
            //     "✅ 빠른 추적 실행: Left Pixels = {}, Right Pixels = {}, Left Detected = {}, Right Detected = {}",
            //     lfx.len(), rfx.len(), l_det, r_det
            // ); // <--- 로그 추가

            // 결과 할당
            sliding_window_img = img;
            left_fitx = lfx;
            right_fitx = rfx;
            left_lane_detected = l_det;
            right_lane_detected = r_det;

            // [실패 처리] 만약 빠른 추적에 실패했다면, 다음 프레임에서 전체 탐색을 하도록 상태를 리셋합니다.
            if !(left_lane_detected || right_lane_detected) {
                // println!("⚠️ 추적 실패! 다음 프레임에서 전체 탐색을 실시합니다.");
                pipeline.left_a.clear();
                pipeline.left_b.clear();
                pipeline.left_c.clear();
                pipeline.right_a.clear();
                pipeline.right_b.clear();
                pipeline.right_c.clear();
            }
        } else {
            // [전체 탐색] 이전 기록이 없으면, sliding_window로 전체 영역을 탐색합니다.
            let (img, lfx, rfx, l_det, r_det) =
                pipeline.sliding_window(&birds_eye_img, 15, 100, 50, true)?;
            // println!(
            //     "🔍 전체 탐색 실행: Left Pixels = {}, Right Pixels = {}, Left Detected = {}, Right Detected = {}",
            //     lfx.len(), rfx.len(), l_det, r_det
            // );

            // 결과 할당
            sliding_window_img = img;
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
        lane_angle_dto = DtoCamLaneAngle::new(steering_angle, alive_cnt);
        event1 = VfbEvent::CamLaneAngleEvent(Arc::new(lane_angle_dto));
        let _ = tx.send(event1.clone());
        let _ = debug.send(event1.clone());

        birds_eye_view_dto = DtoCamBirdEyeView::new(Arc::new(birds_eye_img), 1280, 720, alive_cnt);
        event2 = VfbEvent::CamBirdEyeViewEvent(Arc::new(birds_eye_view_dto));
        let _ = tx.send(event2.clone());
        let _ = debug.send(event2.clone());

        alive_cnt += 1;
    }
}