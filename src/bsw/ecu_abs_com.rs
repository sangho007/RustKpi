use crate::calibration::com::ComCalibration;
use crate::rte::rte_dto::DtoTcpTelemetry;
use crate::rte::rte_main::TcpChannels;
use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use tokio::io::{AsyncReadExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::task::JoinHandle;

/// 아이폰이 USB 터널을 통해 전송하는 IMU 스트림을 수신한다.
/// 리스너는 상시 대기하며 새 연결이 수립되면 패킷을 읽어 RTE 채널로 전달한다.
pub async fn ea_usb_tcp_gateway(channels: TcpChannels) {
    let calibration = ComCalibration::default();
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

    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    let alive_counter = Arc::new(AtomicU32::new(0));
    let mut workers: Vec<JoinHandle<()>> = Vec::new();

    loop {
        tokio::select! {
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
            discard_payload(&mut reader, frame_len).await?;
            continue;
        }

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

async fn discard_payload(
    reader: &mut BufReader<TcpStream>,
    mut remaining: usize,
) -> io::Result<()> {
    let mut buffer = vec![0u8; 4096];
    while remaining > 0 {
        let chunk = remaining.min(buffer.len());
        reader.read_exact(&mut buffer[..chunk]).await?;
        remaining -= chunk;
    }
    Ok(())
}
