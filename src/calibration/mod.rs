//! 시스템 전반에서 사용하는 캘리브레이션 데이터 모음.
//! 하드웨어·알고리즘 매개변수를 구조체로 정의하고 기본값을 제공한다.

pub mod adas_cod;
pub mod adas_localization;
pub mod com;
pub mod forward_collision;
pub mod lane;
pub mod pwm;
pub mod traffic_light;
pub mod ultrasonic;

pub use adas_cod::{AdasLateralCalibration, AdasLongitudinalCalibration};
pub use adas_localization::{
    LOCALIZATION_MAP_PRESETS, LocalizationDestination, LocalizationLane, LocalizationMapId,
    LocalizationMapPreset, LocalizationStart,
};
pub use lane::{LaneCalibration, LaneCalibrationPreset, camera};
