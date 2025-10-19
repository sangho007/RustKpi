// main.rs

mod asw;
mod bsw;
mod calibration;
mod util;
mod rte;
mod main_runtime;

use opencv::core;

/// 애플리케이션의 비동기 메인 진입점으로 각 ECU 및 ASW 작업을 실행한다.
#[tokio::main(flavor = "multi_thread")]
async fn main() -> opencv::Result<()> {
    // OpenCL 가속을 초기화해 영상 처리 성능을 확보한다.
    if let Err(err) = core::set_use_opencl(true) {
        // 초기화에 실패해도 실행은 계속되므로 경고만 남긴다.
        eprintln!("[INIT] Failed to enable OpenCL: {err}");
    }

    // RTE 시스템에서 공유 채널을 준비한다.
    let rte::rte_main::RteSystem { channels } = rte::rte_main::init();
    let camera_channels = channels.camera.clone();

    // 백그라운드 태스크 핸들을 추적한다.
    let mut tasks = Vec::new();

    // BSW Cam Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = bsw::ecu_abs_cam::ea_cam_provider(cam).await {
                eprintln!("[BSW] Camera provider exited with error: {err}");
            }
        }));
    }

    // BSW UltraSonic Task
    {
        let ultrasonic = channels.ultrasonic.clone();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(ultrasonic).await;
        }));
    }

    // BSW Pwm Task
    {
        let control = channels.control.clone();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_pwm::ea_pca9685_actuator("PCA9685", control).await;
        }));
    }

    // ASW PreProcess Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_pre_processing("PreProcess", cam).await {
                eprintln!("[ASW] Lane pre-processing exited with error: {err}");
            }
        }));
    }

    // ASW LaneAngle Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_get_lane_angle("LaneAngle", cam).await {
                eprintln!("[ASW] Lane angle exited with error: {err}");
            }
        }));
    }

    // ASW TrafficLight Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_trafficlight::runnable_trafficlight_detection("Traffic", cam).await {
                eprintln!("[ASW] Traffic light detection exited with error: {err}");
            }
        }));
    }
    // ASW ForwardCollision Task
    {
        let ultrasonic = channels.ultrasonic.clone();
        tasks.push(tokio::spawn(async move {
            asw::forwardcollision_ultrasonic::runnable_obstacle_detection("ForwardCollision", ultrasonic).await;
        }));
    }

    // ADAS Lateral Control Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_control::runnable_adas_lateral("ADAS-Lateral", chans).await;
        }));
    }

    // GUI 및 신호 처리 루프를 실행한다.
    let result = main_runtime::run(channels).await;

    // 실행 중이던 태스크를 정리한다.
    for handle in tasks {
        handle.abort();
        let _ = handle.await;
    }

    // 런타임 결과에 따라 종료 코드를 선택한다.
    let exit_code = match result {
        Ok(_) => 0,
        Err(err) => {
            eprintln!("[MAIN] runtime error: {}", err);
            1
        }
    };

    // 운영체제에 종료 코드를 전달한다.
    std::process::exit(exit_code);
}
