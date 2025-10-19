// 파일명: ecu_abs_pwm

use crate::bsw::lib::pwm_lib::*;
use crate::calibration::pwm::PwmCalibration;
use crate::rte::rte_main::ControlChannels;
use linux_embedded_hal::I2cdev;
use std::time::Instant;

use pwm_pca9685::{Address, Pca9685};
use tokio::{select, sync::broadcast::error::RecvError};

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
        let _ = pwm.set_channel_off(*channel, angle_to_pwm(*default_angle));
    }

    let mut servo_rx = control.servo_tx.subscribe();
    let mut dc_rx = control.dc_motor_tx.subscribe();
    let mut servo_state: Vec<Option<u32>> = pwm_calibration
        .servo_default_angles
        .iter()
        .map(|&angle| Some(angle))
        .collect();
    let mut last_servo_log = Instant::now();
    let mut last_dc_state: Option<(u32, u32)> = None;
    let mut last_dc_log = Instant::now();

    // --- 들어오는 명령어를 처리하는 메인 루프 ---
    loop {
        select! {
            servo_result = servo_rx.recv() => {
                match servo_result {
                    Ok(servo_dto) => {
                        if let Some(&target_channel) = pwm_calibration.servo_channels.get(servo_dto.channel as usize) {
                            let pwm_val = angle_to_pwm(servo_dto.angle);
                            if let Err(e) = pwm.set_channel_off(target_channel, pwm_val) {
                                eprintln!("[BSW] 서보 채널 {:?} OFF 값 설정 실패: {:?}", target_channel, e);
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
