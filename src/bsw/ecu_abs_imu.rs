use crate::bsw::lib::imu_proto;
use crate::rte::rte_main::ImuChannels;
use std::sync::Arc;
use tokio::sync::broadcast::error::RecvError;

/// TCP로 수신한 프로토버퍼 IMU 패킷을 파싱해 ASW에서 소비 가능한 DTO로 변환한다.
pub async fn ea_imu_telemetry(channels: ImuChannels) {
    let mut rx = channels.raw_tx.subscribe();
    let imu_tx = channels.parsed_tx.clone();

    loop {
        match rx.recv().await {
            Ok(raw) => match imu_proto::decode_imu(raw.payload.as_ref(), raw.alive_cnt) {
                Ok(dto) => {
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
