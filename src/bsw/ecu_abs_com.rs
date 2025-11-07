//! BSW 통신(Com) ECU.
//! USB를 통해 들어오는 TCP 스트림을 수신하고, IMU 원시 데이터를 RTE로 전달한다.
//! 여러 클라이언트 연결을 동시에 처리하며, 각 연결은 별도의 비동기 태스크로 구동된다.

use crate::calibration::com::ComCalibration;
use crate::rte::rte_dto::DtoTcpTelemetry;
use crate::rte::rte_main::TcpChannels;
use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use tokio::io::{AsyncReadExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::watch;
use tokio::task::JoinHandle;

/// 아이폰이 USB 터널을 통해 전송하는 IMU 스트림을 수신한다.
/// 리스너는 상시 대기하며 새 연결이 수립되면 패킷을 읽어 RTE 채널로 전달한다.
pub async fn ea_usb_tcp_gateway(channels: TcpChannels, shutdown: &mut watch::Receiver<bool>) {
    let calibration = ComCalibration::default();
    // 아이폰에서 전송되는 USB 터널링 포트를 수신하기 위해 지정된 주소로 바인딩한다.
    let listener = match TcpListener::bind((calibration.tcp_host, calibration.tcp_port)).await {
        Ok(listener) => {
            println!(
                "[BSW][COM] Listening on {}:{}",
                calibration.tcp_host, calibration.tcp_port
            );
            listener
        }
        Err(err) => {
            eprintln!(
                "[BSW][COM] Failed to bind {}:{}: {}. TCP gateway will not start.",
                calibration.tcp_host, calibration.tcp_port, err
            );
            return;
        }
    };

    // 모든 연결에서 공유하는 Alive 카운터. 프레임 순서를 추적한다.
    let alive_counter = Arc::new(AtomicU32::new(0));
    let mut workers: Vec<JoinHandle<()>> = Vec::new();

    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    loop {
        tokio::select! {
            _ = shutdown.changed() => {
                if *shutdown.borrow() {
                    println!("[BSW][COM] Shutdown signal received, stopping TCP gateway.");
                    break;
                } else {
                    continue;
                }
            }
            _ = &mut ctrl_c => {
                println!("[BSW][COM] Ctrl-C received, shutting down TCP gateway.");
                break;
            }
            accept_result = listener.accept() => {
                match accept_result {
                    Ok((stream, addr)) => {
                        println!("[BSW][COM] Accepted TCP connection from {}", addr);
                        let chans = channels.clone();
                        let counter = alive_counter.clone();
                        let max_payload_size = calibration.max_payload_size;
                        // 각 클라이언트를 독립 태스크로 분리해 백프레셔 없이 처리한다.
                        let handle = tokio::spawn(async move {
                            if let Err(err) =
                                stream_telemetry(stream, chans, counter, addr, max_payload_size).await
                            {
                                eprintln!("[BSW][COM] Connection {} ended with error: {}", addr, err);
                            } else {
                                println!("[BSW][COM] Connection {} closed.", addr);
                            }
                        });
                        workers.push(handle);
                    }
                    Err(err) => {
                        eprintln!("[BSW][COM] Failed to accept connection: {}", err);
                    }
                }
            }
        }
    }

    for worker in workers {
        worker.abort();
        let _ = worker.await;
    }

    println!("[BSW][COM] TCP gateway stopped.");
}

/// 단일 TCP 연결에서 프로토콜에 따라 텔레메트리를 읽고 브로드캐스트한다.
/// - 프레임은 [길이(4바이트 Big-Endian)] + [페이로드] 형식이다.
/// - 최대 페이로드 크기를 초과하면 데이터를 폐기하고 연결은 유지한다.
async fn stream_telemetry(
    stream: TcpStream,
    channels: TcpChannels,
    alive_counter: Arc<AtomicU32>,
    addr: SocketAddr,
    max_payload_size: usize,
) -> io::Result<()> {
    let mut reader = BufReader::new(stream);
    let tx = channels.telemetry_tx.clone();

    loop {
        let mut len_buf = [0u8; 4];
        // 먼저 메시지 길이를 읽어 실제 데이터 크기를 파악한다.
        if let Err(err) = reader.read_exact(&mut len_buf).await {
            return Err(err);
        }

        let frame_len = u32::from_be_bytes(len_buf) as usize;
        if frame_len == 0 {
            continue;
        }

        if frame_len > max_payload_size {
            eprintln!(
                "[BSW][COM] Payload length {} from {} exceeds limit {}, dropping frame.",
                frame_len, addr, max_payload_size
            );
            // 과도한 데이터는 지정된 바이트 수만큼 건너뛴 후 다음 프레임으로 넘어간다.
            discard_payload(&mut reader, frame_len).await?;
            continue;
        }

        // 안전한 버퍼를 확보한 뒤 전체 페이로드를 메모리에 읽어들인다.
        let mut payload = vec![0u8; frame_len];
        reader.read_exact(&mut payload).await?;

        let alive_cnt = alive_counter.fetch_add(1, Ordering::Relaxed);
        let telemetry = Arc::new(DtoTcpTelemetry::new(payload, alive_cnt));

        if let Err(err) = tx.send(telemetry) {
            eprintln!(
                "[BSW][COM] Failed to broadcast telemetry from {}: {}",
                addr, err
            );
        }
    }
}

/// 지정된 크기만큼 페이로드를 폐기해 다음 메시지 경계로 이동한다.
async fn discard_payload(
    reader: &mut BufReader<TcpStream>,
    mut remaining: usize,
) -> io::Result<()> {
    let mut buffer = vec![0u8; 4096];
    while remaining > 0 {
        let chunk = remaining.min(buffer.len());
        // 남은 크기만큼 반복해서 읽어 버퍼 내용을 버린다.
        reader.read_exact(&mut buffer[..chunk]).await?;
        remaining -= chunk;
    }
    Ok(())
}
