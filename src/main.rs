// main.rs

mod rte;
mod bsw;
mod asw;

use crate::rte::rte_dto::*;
use crate::rte::rte_main::RteSystem;
use opencv::highgui;
use tokio::task;
use tokio::{select, sync::broadcast::error::RecvError};
use std::sync::Arc;

#[tokio::main(flavor = "multi_thread")]
async fn main() -> opencv::Result<()> {
    let RteSystem { channels } = rte::rte_main::init();
    let camera_channels = channels.camera.clone();
    let ultrasonic_channels = channels.ultrasonic.clone();
    let control_channels = channels.control.clone();

    // BSW Task 생성
    tokio::spawn(bsw::ecu_abs_cam::ea_cam_provider(camera_channels.clone()));
    tokio::spawn(bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(
        ultrasonic_channels.clone(),
    ));
    tokio::spawn(bsw::ecu_abs_pwm::ea_pca9685_actuator(
        "MotorControl",
        control_channels.clone(),
    ));


    // ASW Task 생성
    tokio::spawn(asw::vs_lane::runnable_pre_processing(
        "PreProcess",
        camera_channels.clone(),
    ));
    tokio::spawn(asw::vs_lane::runnable_get_lane_angle(
        "LaneAngle",
        camera_channels.clone(),
    ));
    tokio::spawn(asw::forwardcollision_ultrasonic::runnable_obstacle_detection(
        "UssObstacle",
        ultrasonic_channels.clone(),
    ));
    tokio::spawn(asw::vs_trafficlight::runnable_trafficlight_detection(
        "TrafficLightDetection",
        camera_channels.clone(),
    ));



    // 디버깅용 코드
    println!("== 시스템 실행 중... (GUI 창에서 'q'를 누르면 종료) ==");

    const DEBUG_ON:bool = true;

    
    let mut processed_rx = camera_channels.processed_tx.subscribe();
    let mut birds_eye_rx = camera_channels.bird_eye_tx.subscribe();
    let mut lane_angle_rx = camera_channels.lane_angle_tx.subscribe();
    let mut distance_rx = ultrasonic_channels.raw_tx.subscribe();

    // GUI를 위한 윈도우 생성
    highgui::named_window("CAM View", highgui::WINDOW_AUTOSIZE)?;

    // 각 창에 표시할 최신 프레임을 저장할 변수 (루프 외부에 선언)
    let mut latest_processed_frame: Option<Arc<DtoCamProcessed>> = None;
    let mut latest_birds_eye_frame: Option<Arc<DtoCamBirdEyeView>> = None;

    // Main 스레드에서 GUI 이벤트 루프 실행
    'main_loop: loop {
        select! {
            result = processed_rx.recv() => match result {
                Ok(cam_processed) => {
                    latest_processed_frame = Some(cam_processed);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Processed frame lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Processed frame channel closed.");
                    break 'main_loop;
                }
            },
            result = birds_eye_rx.recv() => match result {
                Ok(birds_eye) => {
                    latest_birds_eye_frame = Some(birds_eye);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Bird eye stream lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Bird eye channel closed.");
                    break 'main_loop;
                }
            },
            result = lane_angle_rx.recv() => match result {
                Ok(lane_angle) => {
                    println!("Angle: {}, alive_cnt: {}", lane_angle.angle, lane_angle.alive_cnt);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Lane angle lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Lane angle channel closed.");
                    break 'main_loop;
                }
            },
            result = distance_rx.recv() => match result {
                Ok(distance) => {
                    println!("distance: {}, alive_cnt: {}", distance.distance, distance.alive_cnt);
                }
                Err(RecvError::Lagged(n)) => {
                    eprintln!("[MAIN] Uss lagged by {}", n);
                }
                Err(RecvError::Closed) => {
                    eprintln!("[MAIN] Uss angle channel closed.");
                    break 'main_loop;
                }
            }
        }
        
        if DEBUG_ON {
            // 2) 블로킹 렌더링을 block_in_place로 감쌈
            let should_quit = task::block_in_place(|| -> opencv::Result<bool> {
                if let Some(frame) = &latest_processed_frame {
                    highgui::imshow("CAM View", &*frame.img)?;
                }
                if let Some(frame) = &latest_birds_eye_frame {
                    highgui::imshow("Bird's Eye View", &*frame.img)?;
                }
                let key = highgui::wait_key(1)?;
                Ok(key == 113) // 'q'
            })?;

            if should_quit {
                break;
            }
        }
    }
    
    println!("== 시스템 실행 중... (Ctrl+C로 종료) ==");
    tokio::signal::ctrl_c().await.expect("Ctrl-C 핸들러 설정 실패");
    println!("\n== 시뮬레이션 종료 ==");
    Ok(())
}
