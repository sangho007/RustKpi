// main.rs

mod rte;
mod bsw;
mod asw;

use std::sync::Arc;
use opencv::highgui;
use crate::rte::rte_dto::{VfbEvent};
use crate::rte::rte_dto::*;

#[tokio::main]
async fn main() -> opencv::Result<()> {
    let vfb_sender = rte::rte_main::init();
    let debug_sender = rte::rte_main::debug_init();

    // BSW Task 생성
    tokio::spawn(bsw::ecu_abs_cam::ea_cam_provider(vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(bsw::ecu_abs_pca9685::ea_pca9685_actuator("MotorControl", vfb_sender.clone()));

    // ASW Task 생성
    tokio::spawn(asw::vs_lane::runnable_pre_processing("PreProcess", vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(asw::vs_lane::runnable_get_lane_angle("LaneAngle", vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(asw::uss_obstacle::runnable_obstacle_detection("UssObstacle", vfb_sender.clone(), debug_sender.clone()));
    tokio::spawn(asw::vs_trafficlight::runnable_trafficlight_detection("TrafficLightDetection", vfb_sender.clone(), debug_sender.clone()));



    // 디버깅용 코드
    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");
    let mut vfb_receiver_main = vfb_sender.subscribe();

    // GUI를 위한 윈도우 생성
    highgui::named_window("CAM View", highgui::WINDOW_AUTOSIZE)?;

    // 각 창에 표시할 최신 프레임을 저장할 변수 (루프 외부에 선언)
    let mut latest_processed_frame: Option<Arc<DtoCamProcessed>> = None;
    let mut latest_birds_eye_frame: Option<Arc<DtoCamBirdEyeView>> = None;

    // Main 스레드에서 GUI 이벤트 루프 실행
    loop {
        match vfb_receiver_main.recv().await {
            Ok(VfbEvent::CamProcessedEvent(cam_processed)) => {
                latest_processed_frame = Some(cam_processed);
            }
            Ok(VfbEvent::CamBirdEyeViewEvent(birds_eye)) => {
                latest_birds_eye_frame = Some(birds_eye);
            }
            Ok(VfbEvent::CamLaneAngleEvent(lane_angle)) => {
                // 데이터는 이전처럼 그냥 출력
                println!("Angle: {}, alive_cnt: {}", lane_angle.angle, lane_angle.alive_cnt);
            }
            Err(e) => {
                //println!("Error: {:?}", e)
            },
            _ => {}
        }

        // 2. 렌더링: 저장된 최신 프레임이 있다면 화면에 표시
        // if let Some(frame) = &latest_processed_frame {
        //     highgui::imshow("CAM View", &*frame.img)?;
        // }
        // if let Some(frame) = &latest_birds_eye_frame {
        //     highgui::imshow("Bird's Eye View", &*frame.img)?;
        // }

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