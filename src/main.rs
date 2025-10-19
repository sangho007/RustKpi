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

    // BSW Task 생성
    tokio::spawn(bsw::ecu_abs_cam::ea_cam_provider(camera_channels.clone()));
    tokio::spawn(bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(
        channels.ultrasonic.clone(),
    ));
    tokio::spawn(bsw::ecu_abs_pwm::ea_pca9685_actuator(
        "PCA9685",
        channels.control.clone(),
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
    tokio::spawn(asw::vs_trafficlight::runnable_trafficlight_detection(
        "Traffic",
        camera_channels.clone(),
    ));
    tokio::spawn(asw::forwardcollision_ultrasonic::runnable_obstacle_detection(
        "ForwardCollision",
        channels.ultrasonic.clone(),
    ));

    main_runtime::run(channels).await
}
