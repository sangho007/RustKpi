//! 통신(TCP) 게이트웨이용 캘리브레이션.
//! USB 터널링 포트와 메시지 크기 제한을 정의한다.

#[derive(Clone, Copy, Debug)]
/// TCP 게이트웨이 설정값.
pub struct ComCalibration {
    pub tcp_host: &'static str,
    pub tcp_port: u16,
    pub max_payload_size: usize,
}

impl Default for ComCalibration {
    fn default() -> Self {
        Self {
            tcp_host: "127.0.0.1",
            tcp_port: 4820,
            max_payload_size: 512 * 1024, // 최대 512KiB까지 허용해 비정상 패킷을 차단
        }
    }
}
