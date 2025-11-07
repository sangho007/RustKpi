// main.rs

mod asw;
mod bsw;
mod calibration;
mod main_runtime;
mod rte;
mod util;

use opencv::core;
use tokio::sync::broadcast::error::RecvError;
use tokio::time::{Duration, sleep};

/// `true`이면 EV READY(초기 IMU 수신) 후 3초 대기 뒤 주행 태스크를 기동한다.
/// 디버그 시 즉시 시작하려면 `false`로 바꾼 뒤 재빌드한다.
const EV_READY_GATE_ENABLED: bool = true;

async fn wait_for_ev_ready(
    channels: &rte::rte_main::RteChannels,
    gate_enabled: bool,
) -> opencv::Result<()> {
    if !gate_enabled {
        println!("[INIT] EV READY gate disabled (debug mode).");
        return Ok(());
    }

    println!("[INIT] Waiting for EV READY (IMU stream)...");
    let mut imu_rx = channels.imu.parsed_tx.subscribe();
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    loop {
        tokio::select! {
            biased;

            ctrl = &mut ctrl_c => {
                match ctrl {
                    Ok(()) => {
                        println!("[INIT] Ctrl-C received before EV READY; aborting startup...");
                        return Err(opencv::Error::new(
                            opencv::core::StsError,
                            "Cancelled while waiting for EV READY".to_string(),
                        ));
                    }
                    Err(err) => {
                        eprintln!("[INIT] Failed to listen for Ctrl-C: {}", err);
                        return Err(opencv::Error::new(
                            opencv::core::StsError,
                            format!("Failed to listen for Ctrl-C: {}", err),
                        ));
                    }
                }
            }

            recv = imu_rx.recv() => match recv {
                Ok(imu) => {
                    println!(
                        "[INIT] EV READY confirmed (seq={}, alive_cnt={}).",
                        imu.header.seq, imu.alive_cnt
                    );
                    println!("[INIT] Stabilizing... (3s)");
                    sleep(Duration::from_secs(3)).await;
                    return Ok(());
                }
                Err(RecvError::Lagged(_)) => {
                    continue;
                }
                Err(RecvError::Closed) => {
                    return Err(opencv::Error::new(
                        opencv::core::StsError,
                        "IMU channel closed before EV READY".to_string(),
                    ));
                }
            }
        }
    }
}

/// 애플리케이션의 비동기 메인 진입점으로 각 BSW 및 ASW 작업을 실행한다.
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
    let gate_enabled = EV_READY_GATE_ENABLED;

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

    // BSW Com Task
    {
        let com = channels.com.clone();
        let mut shutdown_rx = channels.shutdown.subscribe();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_com::ea_usb_tcp_gateway(com, &mut shutdown_rx).await;
        }));
    }

    // BSW IMU Task
    {
        let imu = channels.imu.clone();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_imu::ea_imu_telemetry(imu).await;
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
        let shutdown = channels.shutdown.subscribe();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_pwm::ea_pca9685_actuator("PCA9685", control, shutdown).await;
        }));
    }

    if let Err(err) = wait_for_ev_ready(&channels, gate_enabled).await {
        eprintln!("[INIT] EV READY wait failed: {}", err);
        for handle in &tasks {
            handle.abort();
        }
        return Err(err);
    }

    if gate_enabled {
        println!("[INIT] EV READY satisfied. Launching driving tasks...");
    } else {
        println!("[INIT] Launching driving tasks immediately (gate disabled).");
    }

    // ASW PreProcess Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_vs_preprocessing("PreProcess", cam).await {
                eprintln!("[ASW] Lane pre-processing exited with error: {err}");
            }
        }));
    }

    // ASW LaneAngle Task
    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_vs_get_lane_angle("LaneAngle", cam).await {
                eprintln!("[ASW] Lane angle exited with error: {err}");
            }
        }));
    }

    // ASW TrafficLight Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) =
                asw::vs_trafficlight::runnable_vs_detect_trafficlight("Traffic", chans).await
            {
                eprintln!("[ASW] Traffic light detection exited with error: {err}");
            }
        }));
    }
    // ASW ForwardCollision Task
    {
        let ultrasonic = channels.ultrasonic.clone();
        tasks.push(tokio::spawn(async move {
            asw::forwardcollision_ultrasonic::runnable_forwardcollision_obstacle_detection(
                "ForwardCollision",
                ultrasonic,
            )
            .await;
        }));
    }

    // ASW Localization Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_localization::runnable_adas_localization("ADAS-Localization", chans).await;
        }));
    }

    // ASW Arrival Detection Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_localization::runnable_adas_arrival("ADAS-Arrival", chans).await;
        }));
    }

    // ASW Global Path Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_path_global::runnable_adas_path_global("ADAS-Path-Global", chans).await;
        }));
    }

    // ASW Local Path Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_path_local::runnable_adas_path_local("ADAS-Path-Local", chans).await;
        }));
    }

    // ASW Local Path Smoothing Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_path_local::runnable_adas_path_smoothing("ADAS-Path-Smooth", chans).await;
        }));
    }

    // ADAS Lateral Control Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_cod::runnable_adas_lateral("ADAS-Lateral", chans).await;
        }));
    }

    // ADAS Longitudinal Control Task
    {
        let chans = channels.clone();
        tasks.push(tokio::spawn(async move {
            asw::adas_cod::runnable_adas_longitudinal("ADAS-Longitudinal", chans).await;
        }));
    }

    // GUI 및 신호 처리 루프를 실행한다.
    let runtime_channels = channels.clone();
    let result = main_runtime::run(runtime_channels).await;
    channels.shutdown.trigger();

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
