// 파일명: ecu_abs_pca9685.rs

use crate::rte::rte_main::{VfbSender};
use crate::rte::rte_dto::{VfbEvent};
use crate::bsw::lib::pca9685_lib::*;
use linux_embedded_hal::I2cdev;

use pwm_pca9685::{Address, Channel, Pca9685};
use tokio::sync::broadcast::error::RecvError;

pub async fn ea_pca9685_actuator(id: &'static str, tx: VfbSender) {
    // --- I2C 및 PCA9685 드라이버 초기화 ---
    let i2c_dev = match I2cdev::new(I2C_BUS) {
        Ok(dev) => dev,
        Err(e) => {
            eprintln!("[BSW] I2C 버스 '{}' 열기 실패: {:?}. 액추에이터를 실행하지 않습니다.", I2C_BUS, e);
            return;
        }
    };

    let address = Address::from(PCA9685_ADDRESS);
    let mut pwm = match Pca9685::new(i2c_dev, address) {
        Ok(p) => {
            println!("[BSW] PCA9685 드라이버 초기화 성공.");
            p
        },
        Err(e) => {
            eprintln!("[BSW] PCA9685 드라이버 초기화 실패: {:?}. 액추에이터를 실행하지 않습니다.", e);
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
    for &channel in SERVO_CHANNELS.iter() {
        if let Err(e) = pwm.set_channel_on(channel, 0) {
            eprintln!("[BSW] 채널 {:?} ON 시간 설정 실패: {:?}", channel, e);
            return;
        }
    }

    // 서보 초기화
    let _ = pwm.set_channel_off(Channel::C0, angle_to_pwm(170));
    let _ = pwm.set_channel_off(Channel::C1, angle_to_pwm(170));
    let _ = pwm.set_channel_off(Channel::C2, angle_to_pwm(90));

    let mut rx = tx.subscribe();

    // --- 들어오는 명령어를 처리하는 메인 루프 ---
    loop {
        match rx.recv().await {
            // 서보 모터 제어 이벤트 처리
            Ok(VfbEvent::ServoCtrlEvent(servo_cmd)) => {
                if let Some(&target_channel) = SERVO_CHANNELS.get(servo_cmd.channel as usize) {
                    let pwm_val = angle_to_pwm(servo_cmd.angle);
                    println!("[BSW] 서보 명령어 수신: 채널 {}, 각도 {}, PWM 설정 값 {}", servo_cmd.channel, servo_cmd.angle, pwm_val);
                    if let Err(e) = pwm.set_channel_off(target_channel, pwm_val) {
                        eprintln!("[BSW] 서보 채널 {:?} OFF 값 설정 실패: {:?}", target_channel, e);
                    }
                } else {
                    eprintln!("[BSW] 잘못된 서보 채널 번호 수신: {}", servo_cmd.channel);
                }
            },

            // DC 모터 제어 이벤트 처리
            Ok(VfbEvent::DcMotorCtrlEvent(dc_cmd)) => {
                println!("[BSW] DC 모터 명령어 수신: 방향 {}, 속도 {}", dc_cmd.direction, dc_cmd.speed);

                match dc_cmd.direction {
                    1 => { // 정방향
                        motor_control(&mut pwm, Motor::M1, Direction::Forward, percent_to_pwm(dc_cmd.speed)); // 최대 속도4095
                        motor_control(&mut pwm, Motor::M2, Direction::Forward, percent_to_pwm(dc_cmd.speed));
                    },
                    2 => { // 역방향
                        motor_control(&mut pwm, Motor::M1, Direction::Backward, percent_to_pwm(dc_cmd.speed));
                        motor_control(&mut pwm, Motor::M2, Direction::Backward, percent_to_pwm(dc_cmd.speed));
                    },
                    0 => { // 정지
                        motor_stop(&mut pwm, Motor::M1);
                        motor_stop(&mut pwm, Motor::M2);
                    },
                    _ => continue,
                }
            },

            Err(RecvError::Lagged(n)) => {
                eprintln!("[{}] Error receiving event: Lagged by {}", id, n);
                continue;
            }
            _ => { // 관심 없는 다른 VfbEvent는 무시
                continue;
            }
        };
    }

    // 종료 시 PWM 컨트롤러를 정상적으로 비활성화
    // let _ = pwm.destroy();
    // println!("[BSW] PCA9685 서보 액추에이터가 종료되었습니다.");
}

