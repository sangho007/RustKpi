//! BSW PWM ECU.
//! PCA9685 보드를 이용해 서보 및 DC 모터를 제어하며, RTE에서 전달된 명령을 실시간으로 적용한다.
//! 초기화·오류 처리 절차를 명확히 하고, 각 명령 흐름을 로그로 추적한다.

use crate::bsw::lib::pwm_lib::*;
use crate::calibration::pwm::PwmCalibration;
use crate::rte::rte_main::ControlChannels;
use linux_embedded_hal::I2cdev;
use std::time::Instant;

use pwm_pca9685::{Address, Channel, Pca9685};
use tokio::{select, signal, sync::broadcast::error::RecvError};

/// PCA9685 기반 액추에이터 제어 태스크를 실행한다.
/// - 서보/모터 채널을 캘리브레이션 정보에 맞춰 초기화한다.
/// - 브로드캐스트 채널로부터 명령을 구독하여 하드웨어에 즉시 반영한다.
/// - Ctrl-C 신호가 들어오면 안전하게 장치를 비활성화한다.
pub async fn ea_pca9685_actuator(id: &'static str, control: ControlChannels) {
    let pwm_calibration = PwmCalibration::default();
    // --- I2C 및 PCA9685 드라이버 초기화 ---
    let i2c_dev = match I2cdev::new(pwm_calibration.i2c_bus) {
        Ok(dev) => dev,
        Err(e) => {
            eprintln!(
                "[BSW] I2C 버스 '{}' 열기 실패: {:?}. 액추에이터를 실행하지 않습니다.",
                pwm_calibration.i2c_bus, e
            );
            return;
        }
    };

    let address = Address::from(pwm_calibration.device_address);
    let mut pwm = match Pca9685::new(i2c_dev, address) {
        Ok(p) => {
            println!("[BSW] PCA9685 드라이버 초기화 성공.");
            p
        }
        Err(e) => {
            eprintln!(
                "[BSW] PCA9685 드라이버 초기화 실패: {:?}. 액추에이터를 실행하지 않습니다.",
                e
            );
            return;
        }
    };

    // --- 서보 제어를 위한 PWM 설정 (50 Hz) ---
    if let Err(e) = pwm.set_prescale(121) {
        eprintln!("[BSW] PCA9685 prescale 설정 실패: {:?}", e);
        return;
    }
    if let Err(e) = pwm.enable() {
        eprintln!("[BSW] PCA9685 활성화 실패: {:?}", e);
        return;
    }

    // 정의된 모든 서보 채널의 ON 시점을 0으로 초기화
    for &channel in pwm_calibration.servo_channels.iter() {
        if let Err(e) = pwm.set_channel_on(channel, 0) {
            eprintln!("[BSW] 채널 {:?} ON 시간 설정 실패: {:?}", channel, e);
            return;
        }
    }

    for (channel, default_angle) in pwm_calibration
        .servo_channels
        .iter()
        .zip(pwm_calibration.servo_default_angles.iter())
    {
        let pwm_value = match *channel {
            Channel::C0 => angle_to_pwm_steer(*default_angle),
            Channel::C2 => angle_to_pwm_ultrasonic(*default_angle),
            _ => angle_to_pwm_steer(*default_angle),
        };
        let _ = pwm.set_channel_off(*channel, pwm_value);
    }

    // DC 모터 채널을 명시적으로 정지시켜 초기 부팅 시 오동작을 방지한다.
    motor_stop(&mut pwm, Motor::M1);
    motor_stop(&mut pwm, Motor::M2);
    println!("[BSW] DC 모터 초기화: M1/M2를 정지 상태로 설정했습니다.");

    let mut servo_rx = control.servo_tx.subscribe();
    let mut dc_rx = control.dc_motor_tx.subscribe();
    // 최근 명령 상태를 유지해 중복 로그를 줄이고, 초기 각도를 보존한다.
    let mut servo_state: Vec<Option<u32>> = pwm_calibration
        .servo_default_angles
        .iter()
        .map(|&angle| Some(angle))
        .collect();
    let mut last_servo_log = Instant::now();
    // 최근 모터 상태 및 로그 시점을 기록해 불필요한 출력과 진동을 줄인다.
    let mut last_dc_state: Option<(u32, u32)> = None;
    let mut last_dc_log = Instant::now();
    let mut ctrl_c_signal = signal::ctrl_c();
    tokio::pin!(ctrl_c_signal);

    // --- 들어오는 명령어를 처리하는 메인 루프 ---
    loop {
        select! {
            ctrl_c_result = &mut ctrl_c_signal => {
                match ctrl_c_result {
                    Ok(()) => println!("[BSW] Ctrl-C 신호 감지, PCA9685 액추에이터를 종료합니다."),
                    Err(e) => eprintln!("[BSW] Ctrl-C 신호 대기 중 오류: {:?}. 강제 종료합니다.", e),
                }
                break;
            }
            servo_result = servo_rx.recv() => {
                match servo_result {
                    Ok(servo_dto) => {
                        if let Some(&target_channel) = pwm_calibration.servo_channels.get(servo_dto.channel as usize) {
                            println!("[BSW] Demand cmd : {}", servo_dto.angle);
                            let pwm_val_steer = angle_to_pwm_steer(servo_dto.angle);
                            let pwm_val_ultrasonic = angle_to_pwm_ultrasonic(servo_dto.angle);

                            // 서보에 전달되는 듀티비를 채널 별 변환 함수로 적용한다.
                            let result = match target_channel {
                                Channel::C0 => pwm.set_channel_off(target_channel, pwm_val_steer),
                                Channel::C2 => pwm.set_channel_off(target_channel, pwm_val_ultrasonic),
                                _ => pwm.set_channel_off(target_channel, pwm_val_steer),
                            };

                            if let Err(e) = result {
                                eprintln!(
                                    "[BSW] 서보 채널 {:?} OFF 값 설정 실패: {:?}",
                                    target_channel, e
                                );
                            }

                            let idx = servo_dto.channel as usize;
                            if idx >= servo_state.len() {
                                servo_state.resize(idx + 1, None);
                            }
                            if let Some(state_slot) = servo_state.get_mut(idx) {
                                let previous = *state_slot;
                                *state_slot = Some(servo_dto.angle);
                                if previous != Some(servo_dto.angle) || last_servo_log.elapsed() >= pwm_calibration.servo_log_interval {
                                    let summary = servo_state.iter().enumerate()
                                        .map(|(channel, angle)| match angle {
                                            Some(a) => format!("C{}={}", channel, a),
                                            None => format!("C{}=--", channel),
                                        })
                                        .collect::<Vec<_>>()
                                        .join(", ");
                                    println!("[BSW] 서보 상태 요약: {}", summary);
                                    last_servo_log = Instant::now();
                                }
                            }
                        } else {
                            eprintln!("[BSW] 잘못된 서보 채널 번호 수신: {}", servo_dto.channel);
                        }
                    }
                    Err(RecvError::Lagged(n)) => {
                        eprintln!("[{}] Servo command lagged by {}", id, n);
                    }
                    Err(RecvError::Closed) => {
                        eprintln!("[{}] Servo command channel closed.", id);
                        break;
                    }
                }
            }
            dc_result = dc_rx.recv() => {
                match dc_result {
                    Ok(dcmotor_dto) => {
                        match dcmotor_dto.direction {
                            1 => { // 정방향
                                // 듀얼 모터 모두 동일한 속도로 구동한다.
                                motor_control(&mut pwm, Motor::M1, Direction::Forward, percent_to_pwm(dcmotor_dto.speed));
                                motor_control(&mut pwm, Motor::M2, Direction::Forward, percent_to_pwm(dcmotor_dto.speed));
                            },
                            2 => { // 역방향
                                motor_control(&mut pwm, Motor::M1, Direction::Backward, percent_to_pwm(dcmotor_dto.speed));
                                motor_control(&mut pwm, Motor::M2, Direction::Backward, percent_to_pwm(dcmotor_dto.speed));
                            },
                            0 => { // 정지
                                motor_stop(&mut pwm, Motor::M1);
                                motor_stop(&mut pwm, Motor::M2);
                            },
                            _ => continue,
                        }

                        let current_state = (dcmotor_dto.direction, dcmotor_dto.speed);
                        let state_changed = last_dc_state.map(|s| s != current_state).unwrap_or(true);
                        if state_changed || last_dc_log.elapsed() >= pwm_calibration.dc_log_interval {
                            println!(
                                "[BSW] DC 모터 상태: 방향 {}, 속도 {}",
                                current_state.0, current_state.1
                            );
                            last_dc_log = Instant::now();
                        }
                        last_dc_state = Some(current_state);
                    }
                    Err(RecvError::Lagged(n)) => {
                        eprintln!("[{}] DC motor command lagged by {}", id, n);
                    }
                    Err(RecvError::Closed) => {
                        eprintln!("[{}] DC motor command channel closed.", id);
                        break;
                    }
                }
            }
        }
    }

    // 종료 시 PWM 컨트롤러를 정상적으로 비활성화
    if let Err(e) = pwm.disable() {
        eprintln!("[BSW] PCA9685 disable 실패: {:?}", e);
    }
    let _ = pwm.destroy();
    println!("[BSW] PCA9685 서보 액추에이터가 종료되었습니다.");
}
