#[derive(Clone, Copy, Debug)]
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
            max_payload_size: 512 * 1024, // protective cap (512 KiB)
        }
    }
}
