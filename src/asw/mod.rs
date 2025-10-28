//! ASW(Application Software) 계층의 핵심 모듈 모음.
//! 인지·판단·제어 로직을 담당하며, BSW/RTE에서 공급한 데이터를 사용해 차량을 제어한다.

pub mod adas_cod;
pub mod adas_localization;
pub mod adas_path_global;
pub mod adas_path_local;
pub mod forwardcollision_ultrasonic;
pub mod lib;
pub mod vs_lane;
pub mod vs_trafficlight;
