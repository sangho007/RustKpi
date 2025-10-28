//! iOS 기반 IMU 텔레메트리 프로토버퍼를 RTE DTO 구조체로 변환하는 유틸리티.
//! - 하위 필드를 안전하게 파싱하고 누락된 값은 기본값으로 대체한다.
//! - 좌표계 보정을 위해 쿼터니언을 오일러 각(yaw, roll, pitch)으로 변환한다.

use crate::rte::rte_dto::{
    DtoImu, DtoImuAcceleration, DtoImuGyro, DtoImuHeader, DtoImuPose, DtoImuStatus, DtoImuVelocity,
};
use prost::Message;

/// 프로토버퍼 IMU 페이로드를 디코딩해 RTE DTO로 변환한다.
/// - optional 필드는 `Option`으로 변환하고, 누락된 값은 기본값을 사용한다.
/// - `alive_cnt`는 상위 계층에서 전달된 패킷 시퀀스 번호를 그대로 복사한다.
pub fn decode_imu(payload: &[u8], alive_cnt: u32) -> Result<DtoImu, prost::DecodeError> {
    let packet = proto::TelemetryMessage::decode(payload)?;

    let header = packet.header.as_ref().map(to_header).unwrap_or_default();
    let status = packet.status.as_ref().map(to_status);
    let pose = packet.pose_world_phone.as_ref().map(to_pose);
    let velocity = packet.velocity.as_ref().map(to_velocity);
    let acceleration = packet.acceleration.as_ref().map(to_acceleration);
    let gyro = packet.gyro.as_ref().map(to_gyro);

    Ok(DtoImu::new(
        header,
        status,
        pose,
        velocity,
        acceleration,
        gyro,
        alive_cnt,
    ))
}

/// 프로토 헤더 메시지를 DTO 헤더로 변환한다.
fn to_header(raw: &proto::Header) -> DtoImuHeader {
    DtoImuHeader {
        stamp_ns: raw.stamp_ns.unwrap_or_default(),
        dt_ns: raw.dt_ns.unwrap_or_default(),
        seq: raw.seq.unwrap_or_default(),
        session_id: normalize_string(raw.session_id.as_ref()),
        clock_domain: normalize_string(raw.clock_domain.as_ref()),
        frame_id: normalize_string(raw.frame_id.as_ref()),
        child_frame_id: normalize_string(raw.child_frame_id.as_ref()),
    }
}

/// 추적 상태 필드를 DTO로 정리한다.
fn to_status(raw: &proto::Status) -> DtoImuStatus {
    DtoImuStatus {
        tracking: normalize_string(raw.tracking.as_ref()),
        tracking_confidence: raw.tracking_confidence,
        num_features: raw.num_features,
        status_reason: normalize_string(raw.status_reason.as_ref()),
        flags: raw.flags.clone(),
    }
}

/// 포즈(위치 및 자세) 정보를 DTO로 변환한다.
fn to_pose(raw: &proto::PoseWorldPhone) -> DtoImuPose {
    let mut pose = DtoImuPose::default();
    if let Some(position) = raw.position.as_ref() {
        pose.position_world = Some(vector3_to_array(position));
    }
    if let Some(orientation) = raw.orientation.as_ref() {
        pose.orientation_quat = Some(quaternion_to_array(orientation));
        pose.orientation_yaw_roll_pitch = pose
            .orientation_quat
            .as_ref()
            .map(quaternion_to_yaw_roll_pitch);
    }
    if let Some(cov) = raw.covariance.as_ref() {
        pose.position_cov = cov.pos.clone();
        pose.orientation_cov = cov.ori.clone();
    }
    pose.valid = raw.valid;
    pose
}

/// 속도 벡터와 공분산을 DTO 포맷으로 매핑한다.
fn to_velocity(raw: &proto::Velocity) -> DtoImuVelocity {
    let mut velocity = DtoImuVelocity::default();
    if let Some(world) = raw.world.as_ref() {
        velocity.world = Some(vector3_to_array(world));
    }
    velocity.source = normalize_string(raw.source.as_ref());
    velocity.covariance = raw.cov.clone();
    velocity.valid = raw.valid;
    velocity
}

/// 가속도 측정값을 DTO 구조체에 채운다.
fn to_acceleration(raw: &proto::Acceleration) -> DtoImuAcceleration {
    let mut accel = DtoImuAcceleration::default();
    if let Some(body) = raw.body_no_gravity.as_ref() {
        accel.body_no_gravity = Some(vector3_to_array(body));
    }
    if let Some(world) = raw.world.as_ref() {
        accel.world = Some(vector3_to_array(world));
    }
    accel.source = normalize_string(raw.source.as_ref());
    accel.covariance = raw.cov.clone();
    accel.valid = raw.valid;
    accel
}

/// 자이로(각속도) 데이터를 DTO로 변환한다.
fn to_gyro(raw: &proto::Gyro) -> DtoImuGyro {
    let mut gyro = DtoImuGyro::default();
    if let Some(body) = raw.body.as_ref() {
        gyro.body = Some(vector3_to_array(body));
    }
    gyro.source = normalize_string(raw.source.as_ref());
    if let Some(bias) = raw.bias.as_ref() {
        gyro.bias = Some(vector3_to_array(bias));
    }
    gyro.covariance = raw.cov.clone();
    gyro.valid = raw.valid;
    gyro
}

/// 프로토콜 벡터를 고정 길이 배열로 변환한다.
fn vector3_to_array(v: &proto::Vector3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// 프로토콜 쿼터니언을 `[x, y, z, w]` 배열로 변환한다.
fn quaternion_to_array(q: &proto::Quaternion) -> [f64; 4] {
    [q.x, q.y, q.z, q.w]
}

/// 쿼터니언을 Yaw-Roll-Pitch 오일러 각으로 변환한다.
/// - 차량 제어 로직에서는 직관적인 각도 표현이 필요하므로 오일러 각을 병행 제공한다.
fn quaternion_to_yaw_roll_pitch(quat: &[f64; 4]) -> [f64; 3] {
    let x = quat[0];
    let y = quat[1];
    let z = quat[2];
    let w = quat[3];

    let roll = {
        // x축 회전(roll)을 계산한다.
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        sinr_cosp.atan2(cosr_cosp)
    };

    let pitch = {
        // y축 회전(pitch)은 아크사인을 사용하며, 특이점에 대비해 값을 제한한다.
        let sinp = 2.0 * (w * y - z * x);
        if sinp.abs() >= 1.0 {
            sinp.signum() * std::f64::consts::FRAC_PI_2
        } else {
            sinp.asin()
        }
    };

    let yaw = {
        // z축 회전(yaw)을 계산한다.
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        siny_cosp.atan2(cosy_cosp)
    };

    [yaw, roll, pitch]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-6;

    fn assert_close(actual: &[f64; 3], expected: &[f64; 3]) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= EPS,
                "expected {:?}, got {:?}",
                expected,
                actual
            );
        }
    }

    #[test]
    fn quaternion_identity_is_zero_euler() {
        let euler = quaternion_to_yaw_roll_pitch(&[0.0, 0.0, 0.0, 1.0]);
        assert_close(&euler, &[0.0, 0.0, 0.0]);
    }

    #[test]
    fn quaternion_yaw_90_deg() {
        let half = (0.5_f64).sqrt();
        let euler = quaternion_to_yaw_roll_pitch(&[0.0, 0.0, half, half]);
        assert_close(&euler, &[FRAC_PI_2, 0.0, 0.0]);
    }

    #[test]
    fn quaternion_roll_90_deg() {
        let half = (0.5_f64).sqrt();
        let euler = quaternion_to_yaw_roll_pitch(&[half, 0.0, 0.0, half]);
        assert_close(&euler, &[0.0, FRAC_PI_2, 0.0]);
    }

    #[test]
    fn quaternion_pitch_90_deg() {
        let half = (0.5_f64).sqrt();
        let euler = quaternion_to_yaw_roll_pitch(&[0.0, half, 0.0, half]);
        assert_close(&euler, &[0.0, 0.0, FRAC_PI_2]);
    }
}

/// 빈 문자열을 `None`으로 정규화해 불필요한 빈 값을 제거한다.
fn normalize_string(value: Option<&String>) -> Option<String> {
    value.and_then(|s| if s.is_empty() { None } else { Some(s.clone()) })
}

#[allow(clippy::derive_partial_eq_without_eq)]
pub(self) mod proto {
    use prost::Message;

    #[derive(Clone, PartialEq, Message)]
    pub struct TelemetryMessage {
        #[prost(string, optional, tag = "1")]
        pub schema_version: Option<String>,
        #[prost(message, optional, tag = "2")]
        pub header: Option<Header>,
        #[prost(message, optional, tag = "3")]
        pub status: Option<Status>,
        #[prost(message, optional, tag = "4")]
        pub pose_world_phone: Option<PoseWorldPhone>,
        #[prost(message, optional, tag = "5")]
        pub velocity: Option<Velocity>,
        #[prost(message, optional, tag = "6")]
        pub acceleration: Option<Acceleration>,
        #[prost(message, optional, tag = "7")]
        pub gyro: Option<Gyro>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Header {
        #[prost(uint64, optional, tag = "1")]
        pub stamp_ns: Option<u64>,
        #[prost(uint64, optional, tag = "2")]
        pub dt_ns: Option<u64>,
        #[prost(uint64, optional, tag = "3")]
        pub seq: Option<u64>,
        #[prost(string, optional, tag = "4")]
        pub session_id: Option<String>,
        #[prost(string, optional, tag = "5")]
        pub clock_domain: Option<String>,
        #[prost(string, optional, tag = "6")]
        pub frame_id: Option<String>,
        #[prost(string, optional, tag = "7")]
        pub child_frame_id: Option<String>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Status {
        #[prost(string, optional, tag = "1")]
        pub tracking: Option<String>,
        #[prost(double, optional, tag = "2")]
        pub tracking_confidence: Option<f64>,
        #[prost(uint64, optional, tag = "3")]
        pub num_features: Option<u64>,
        #[prost(string, optional, tag = "4")]
        pub status_reason: Option<String>,
        #[prost(string, repeated, tag = "5")]
        pub flags: Vec<String>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct PoseWorldPhone {
        #[prost(message, optional, tag = "1")]
        pub position: Option<Vector3>,
        #[prost(message, optional, tag = "2")]
        pub orientation: Option<Quaternion>,
        #[prost(message, optional, tag = "3")]
        pub covariance: Option<PoseCovariance>,
        #[prost(bool, optional, tag = "4")]
        pub valid: Option<bool>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct PoseCovariance {
        #[prost(double, repeated, packed = "true", tag = "1")]
        pub pos: Vec<f64>,
        #[prost(double, repeated, packed = "true", tag = "2")]
        pub ori: Vec<f64>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Vector3 {
        #[prost(double, tag = "1")]
        pub x: f64,
        #[prost(double, tag = "2")]
        pub y: f64,
        #[prost(double, tag = "3")]
        pub z: f64,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Quaternion {
        #[prost(double, tag = "1")]
        pub x: f64,
        #[prost(double, tag = "2")]
        pub y: f64,
        #[prost(double, tag = "3")]
        pub z: f64,
        #[prost(double, tag = "4")]
        pub w: f64,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Velocity {
        #[prost(message, optional, tag = "1")]
        pub world: Option<Vector3>,
        #[prost(string, optional, tag = "2")]
        pub source: Option<String>,
        #[prost(double, repeated, packed = "true", tag = "3")]
        pub cov: Vec<f64>,
        #[prost(bool, optional, tag = "4")]
        pub valid: Option<bool>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Acceleration {
        #[prost(message, optional, tag = "1")]
        pub body_no_gravity: Option<Vector3>,
        #[prost(message, optional, tag = "2")]
        pub world: Option<Vector3>,
        #[prost(string, optional, tag = "3")]
        pub source: Option<String>,
        #[prost(double, repeated, packed = "true", tag = "4")]
        pub cov: Vec<f64>,
        #[prost(bool, optional, tag = "5")]
        pub valid: Option<bool>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Gyro {
        #[prost(message, optional, tag = "1")]
        pub body: Option<Vector3>,
        #[prost(string, optional, tag = "2")]
        pub source: Option<String>,
        #[prost(message, optional, tag = "3")]
        pub bias: Option<Vector3>,
        #[prost(double, repeated, packed = "true", tag = "4")]
        pub cov: Vec<f64>,
        #[prost(bool, optional, tag = "5")]
        pub valid: Option<bool>,
    }
}
