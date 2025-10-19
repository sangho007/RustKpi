// main.rs

mod asw;
mod bsw;
mod calibration;
mod util;
mod rte;
mod main_runtime;

use opencv::core;

#[tokio::main(flavor = "multi_thread")]
async fn main() -> opencv::Result<()> {
    if let Err(err) = core::set_use_opencl(true) {
        eprintln!("[INIT] Failed to enable OpenCL: {err}");
    }

    let rte::rte_main::RteSystem { channels } = rte::rte_main::init();
    let camera_channels = channels.camera.clone();

    let mut tasks = Vec::new();

    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = bsw::ecu_abs_cam::ea_cam_provider(cam).await {
                eprintln!("[BSW] Camera provider exited with error: {err}");
            }
        }));
    }

    {
        let ultrasonic = channels.ultrasonic.clone();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(ultrasonic).await;
        }));
    }

    {
        let control = channels.control.clone();
        tasks.push(tokio::spawn(async move {
            bsw::ecu_abs_pwm::ea_pca9685_actuator("PCA9685", control).await;
        }));
    }

    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_pre_processing("PreProcess", cam).await {
                eprintln!("[ASW] Lane pre-processing exited with error: {err}");
            }
        }));
    }

    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) = asw::vs_lane::runnable_get_lane_angle("LaneAngle", cam).await {
                eprintln!("[ASW] Lane angle exited with error: {err}");
            }
        }));
    }

    {
        let cam = camera_channels.clone();
        tasks.push(tokio::spawn(async move {
            if let Err(err) =
                asw::vs_trafficlight::runnable_trafficlight_detection("Traffic", cam).await
            {
                eprintln!("[ASW] Traffic light detection exited with error: {err}");
            }
        }));
    }

    {
        let ultrasonic = channels.ultrasonic.clone();
        tasks.push(tokio::spawn(async move {
            asw::forwardcollision_ultrasonic::runnable_obstacle_detection(
                "ForwardCollision",
                ultrasonic,
            )
            .await;
        }));
    }

    let result = main_runtime::run(channels).await;

    for handle in tasks {
        handle.abort();
    }

    result
}
