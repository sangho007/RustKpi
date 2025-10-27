//! BSW(Basic Software) 레이어의 ECU 구현을 모아 둔 모듈입니다.
//! 하드웨어 장치를 직접 제어하거나 원시 데이터를 수집해 상위 레이어(RTE, ASW)로 전달합니다.

pub mod ecu_abs_cam;
pub mod ecu_abs_com;
pub mod ecu_abs_imu;
pub mod ecu_abs_pwm;
pub mod ecu_abs_ultrasonic;
pub mod lib;
