// main.rs

mod Rte;
mod Bsw;
mod Asw;

use std::sync::Arc;
use tokio;
use opencv::highgui;
use crate::Rte::Rte_Dto::{VfbEvent, Dto_CamRaw};
use crate::Rte::Rte_Dto::*;

#[tokio::main]
async fn main() -> opencv::Result<()> {
    let vfb_sender = Rte::Rte_Main::init();
    let debug_sender = Rte::Rte_Main::debug_init();

    // BSW Task 생성
    tokio::spawn(Bsw::EcuAbs_Cam::EA_CamProvider(vfb_sender.clone(), debug_sender.clone()));

    // ASW Task 생성 (GUI 코드 없음)
    tokio::spawn(Asw::Vision::Runnable_PreProcessing("PreProcess", vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(Asw::Vision::Runnable_GetLaneAngle("LaneAngle", vfb_sender.clone(), debug_sender.clone()));



    // 디버깅용 코드
    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");
    // let mut debug_receiver_main = debug_sender.subscribe();
    let mut vfb_receiver_main = vfb_sender.subscribe();

    // GUI를 위한 윈도우 생성
    highgui::named_window("CAM View", highgui::WINDOW_AUTOSIZE)?;

    // 각 창에 표시할 최신 프레임을 저장할 변수 (루프 외부에 선언)
    let mut latest_processed_frame: Option<Arc<Dto_CamProcessed>> = None;
    let mut latest_birds_eye_frame: Option<Arc<Dto_CamBirdEyeView>> = None;

    // Main 스레드에서 GUI 이벤트 루프 실행
    loop {
        // match debug_receiver_main.recv().await {
        match vfb_receiver_main.recv().await {
            Ok(VfbEvent::CamProcessedData(cam_processed)) => {
                latest_processed_frame = Some(cam_processed);
            }
            Ok(VfbEvent::CamCamBirdEyeViewData(birds_eye)) => {
                latest_birds_eye_frame = Some(birds_eye);
            }
            Ok(VfbEvent::CamLaneAngleData(lane_angle)) => {
                // 데이터는 이전처럼 그냥 출력
                println!("Angle: {}, alive_cnt: {}", lane_angle.angle, lane_angle.alive_cnt);
            }
            Err(e) => {
                //println!("Error: {:?}", e)
            },
            _ => {}
        }

        // 2. 렌더링: 저장된 최신 프레임이 있다면 화면에 표시
        if let Some(frame) = &latest_processed_frame {
            highgui::imshow("CAM View", &*frame.img)?;
        }
        if let Some(frame) = &latest_birds_eye_frame {
            highgui::imshow("Bird's Eye View", &*frame.img)?;
        }

        // wait_key를 호출해야 실제로 창이 업데이트되고 키 입력을 받을 수 있습니다.
        // 1ms 대기하며, 'q' 키(ASCII 113)가 입력되면 루프를 탈출합니다.
        let key = highgui::wait_key(1)?;
        if key == 113 { // 'q' key
            break;
        }
    }
    println!("== 시스템 실행 중... (Ctrl+C로 종료) ==");
    tokio::signal::ctrl_c().await.expect("Ctrl-C 핸들러 설정 실패");
    println!("\n== 시뮬레이션 종료 ==");
    Ok(())
}