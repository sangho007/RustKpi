//! 초음파 센서에 대한 캘리브레이션 헬퍼.
//! 현재는 기본 설정만 반환하지만, 향후 동적 로딩을 고려해 별도의 모듈로 분리했다.

use crate::calibration::ultrasonic::UltrasonicCalibration;

/// 초음파 센서 캘리브레이션 값을 반환한다.
pub fn ultrasonic_calibration() -> UltrasonicCalibration {
    UltrasonicCalibration::default()
}
