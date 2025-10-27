//! BSW IMU ECU.
//! TCP로 전달된 프로토버퍼 메시지를 파싱해 ASW가 사용 가능한 DTO로 변환한다.
//! 파싱 실패나 누락된 패킷을 감시하여 로그로 남긴다.

use crate::bsw::lib::imu_proto;
use crate::rte::rte_main::ImuChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

/// TCP로 수신한 프로토버퍼 IMU 패킷을 파싱해 ASW에서 소비 가능한 DTO로 변환한다.
pub async fn ea_imu_telemetry(channels: ImuChannels) {
    let mut rx = channels.raw_tx.subscribe();
    let imu_tx = channels.parsed_tx.clone();

    // 브로드캐스트 채널에서 원시 IMU 프레임을 계속 수신한다.
    loop {
        match rx.recv().await {
            Ok(raw) => match imu_proto::decode_imu(raw.payload.as_ref(), raw.alive_cnt) {
                Ok(dto) => {
                    // 파싱된 값을 ASW에서 공유할 수 있도록 Arc로 감싼 뒤 브로드캐스트한다.
                    let _ = imu_tx.send(Arc::new(dto));
                }
                Err(err) => {
                    eprintln!(
                        "[BSW][IMU] 프로토버퍼 디코딩 실패(alive_cnt={}): {}",
                        raw.alive_cnt, err
                    );
                }
            },
            Err(RecvError::Lagged(skipped)) => {
                eprintln!(
                    "[BSW][IMU] IMU 파서가 {}개의 텔레메트리 프레임을 놓쳤습니다.",
                    skipped
                );
            }
            Err(RecvError::Closed) => {
                println!("[BSW][IMU] 원시 텔레메트리 채널이 종료되어 IMU 파서를 정지합니다.");
                break;
            }
        }
    }
}
