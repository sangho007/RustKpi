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
    if let Err(err) = core::set_use_opencl(true) { // OpenCL 사용을 시도하여 가속을 활성화한다.
        eprintln!("[INIT] Failed to enable OpenCL: {err}"); // OpenCL 설정 실패 시 오류 메시지를 출력한다.
    } // OpenCL 초기화 블록을 종료한다.

    let rte::rte_main::RteSystem { channels } = rte::rte_main::init(); // RTE 시스템을 초기화하고 채널 집합을 획득한다.
    let camera_channels = channels.camera.clone(); // 카메라 채널을 복제하여 여러 작업에서 공유한다.

    let mut tasks = Vec::new(); // 비동기 작업 핸들을 저장할 벡터를 초기화한다.

    {
        let cam = camera_channels.clone(); // 카메라 채널을 해당 작업용으로 복제한다.
        tasks.push(tokio::spawn(async move { // 카메라 공급자 작업을 새 비동기 태스크로 실행한다.
            if let Err(err) = bsw::ecu_abs_cam::ea_cam_provider(cam).await { // 카메라 프레임 공급을 시작하고 실패 시 결과를 확인한다.
                eprintln!("[BSW] Camera provider exited with error: {err}"); // 카메라 공급 실패 시 오류를 기록한다.
            } // 카메라 공급 작업 에러 처리 블록을 종료한다.
        })); // 생성한 작업 핸들을 벡터에 저장한다.
    } // 카메라 공급자 작업 설정 블록을 종료한다.

    {
        let ultrasonic = channels.ultrasonic.clone(); // 초음파 채널을 해당 작업용으로 복제한다.
        tasks.push(tokio::spawn(async move { // 초음파 공급자를 비동기 태스크로 실행한다.
            bsw::ecu_abs_ultrasonic::ea_ultrasonic_provider(ultrasonic).await; // 초음파 센서 데이터를 지속적으로 전송한다.
        })); // 생성한 초음파 태스크 핸들을 벡터에 추가한다.
    } // 초음파 공급자 작업 설정 블록을 종료한다.

    {
        let control = channels.control.clone(); // 제어 채널을 해당 작업용으로 복제한다.
        tasks.push(tokio::spawn(async move { // PWM 액추에이터 작업을 비동기 태스크로 실행한다.
            bsw::ecu_abs_pwm::ea_pca9685_actuator("PCA9685", control).await; // PCA9685 제어 루틴을 수행한다.
        })); // 생성한 액추에이터 태스크 핸들을 벡터에 추가한다.
    } // 액추에이터 작업 설정 블록을 종료한다.

    {
        let cam = camera_channels.clone(); // 차선 처리용 카메라 채널을 복제한다.
        tasks.push(tokio::spawn(async move { // 전처리 작업을 비동기 태스크로 실행한다.
            if let Err(err) = asw::vs_lane::runnable_pre_processing("PreProcess", cam).await { // 차선 전처리 실행 결과를 확인한다.
                eprintln!("[ASW] Lane pre-processing exited with error: {err}"); // 전처리 실패 시 오류를 기록한다.
            } // 전처리 에러 처리 블록을 종료한다.
        })); // 전처리 태스크 핸들을 벡터에 추가한다.
    } // 전처리 작업 설정 블록을 종료한다.

    {
        let cam = camera_channels.clone(); // 차선 각도 계산용 카메라 채널을 복제한다.
        tasks.push(tokio::spawn(async move { // 차선 각도 계산 작업을 비동기 태스크로 실행한다.
            if let Err(err) = asw::vs_lane::runnable_get_lane_angle("LaneAngle", cam).await { // 차선 각도 계산 결과를 확인한다.
                eprintln!("[ASW] Lane angle exited with error: {err}"); // 차선 각도 계산 실패 시 오류를 기록한다.
            } // 차선 각도 에러 처리 블록을 종료한다.
        })); // 차선 각도 태스크 핸들을 벡터에 추가한다.
    } // 차선 각도 작업 설정 블록을 종료한다.

    {
        let cam = camera_channels.clone(); // 신호등 탐지용 카메라 채널을 복제한다.
        tasks.push(tokio::spawn(async move { // 신호등 탐지 작업을 비동기 태스크로 실행한다.
            if let Err(err) = asw::vs_trafficlight::runnable_trafficlight_detection("Traffic", cam).await { // 신호등 탐지 실행 결과를 확인한다.
                eprintln!("[ASW] Traffic light detection exited with error: {err}"); // 신호등 탐지 실패 시 오류를 기록한다.
            } // 신호등 탐지 에러 처리 블록을 종료한다.
        })); // 신호등 탐지 태스크 핸들을 벡터에 추가한다.
    } // 신호등 탐지 작업 설정 블록을 종료한다.

    {
        let ultrasonic = channels.ultrasonic.clone(); // 전방 충돌 감지용 초음파 채널을 복제한다.
        tasks.push(tokio::spawn(async move { // 전방 충돌 감지 작업을 비동기 태스크로 실행한다.
            asw::forwardcollision_ultrasonic::runnable_obstacle_detection("ForwardCollision", ultrasonic).await; // 전방 충돌 루틴을 실행하고 완료될 때까지 대기한다.
        })); // 전방 충돌 태스크 핸들을 벡터에 추가한다.
    } // 전방 충돌 작업 설정 블록을 종료한다.

    let result = main_runtime::run(channels).await; // 런타임 루프를 실행하고 결과를 획득한다.

    for handle in tasks { // 모든 백그라운드 태스크를 순회한다.
        handle.abort(); // 태스크를 중단시킨다.
        let _ = handle.await; // 중단된 태스크가 종료될 때까지 대기한다.
    } // 태스크 정리 루프를 종료한다.

    let exit_code = match result { // 런타임 실행 결과에 따라 종료 코드를 결정한다.
        Ok(_) => 0, // 성공 시 종료 코드를 0으로 설정한다.
        Err(err) => { // 오류가 발생한 경우를 처리한다.
            eprintln!("[MAIN] runtime error: {}", err); // 런타임 오류 메시지를 출력한다.
            1 // 오류 발생 시 종료 코드를 1로 설정한다.
        } // 오류 매치 블록을 종료한다.
    }; // 종료 코드 매칭 블록을 종료한다.

    std::process::exit(exit_code); // 프로그램을 지정한 종료 코드로 종료한다.
}
