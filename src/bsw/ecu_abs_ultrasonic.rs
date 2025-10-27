//! BSW 초음파 ECU.
//! HC-SR04 센서에서 주기적으로 거리를 측정해 RTE 채널로 전달하고,
//! 측정 품질을 추적해 장기적인 상태를 로그로 남긴다.

use crate::bsw::lib::ultrasonic_lib::*;
use crate::rte::rte_dto::DtoUltraSonicRaw;
use crate::rte::rte_main::UltrasonicChannels;
use hc_sr04::{HcSr04, Unit};
use std::sync::Arc;
use std::time::Instant;
use tokio::time;

/// 초음파 센서에서 거리를 측정해 RTE RAW 채널로 퍼블리시한다.
pub async fn ea_ultrasonic_provider(channels: UltrasonicChannels) {
    let calibration = ultrasonic_calibration();
    let mut alive_cnt = 0;
    // 센서 샘플링 간격을 유지하기 위해 Tokio interval을 사용한다.
    let mut interval = time::interval(calibration.sample_interval);
    let UltrasonicChannels { raw_tx, .. } = channels;

    // 센서 초기화는 실패하면 더 이상 진행할 수 없으므로, 에러를 출력하고 함수를 종료합니다.
    let mut sensor = match HcSr04::new(calibration.trigger_pin, calibration.echo_pin, None) {
        Ok(s) => {
            println!("[BSW] 초음파 센서 초기화 성공");
            s // 성공하면 센서 객체를 반환
        }
        Err(e) => {
            eprintln!(
                "[BSW] 초음파 센서 초기화 실패: {:?}. Provider를 종료합니다.",
                e
            );
            return; // 함수 종료
        }
    };

    let mut last_log = Instant::now();
    // 일정 시간 동안 정상/범위 초과 측정 횟수를 누적해 품질을 모니터링한다.
    let mut in_range_samples: u32 = 0;
    let mut out_of_range_samples: u32 = 0;
    let mut last_distance: Option<f32> = None;

    // 센서에서 값을 읽어 DTO로 전달하는 메인 루프.
    loop {
        interval.tick().await;

        // 측정 실패는 일시적일 수 있으므로, 에러만 출력하고 루프는 계속 진행합니다.
        match sensor.measure_distance(Unit::Centimeters) {
            Ok(Some(distance)) => {
                // 측정 성공
                let ultrasonic_raw = Arc::new(DtoUltraSonicRaw::new(distance, alive_cnt));
                let _ = raw_tx.send(ultrasonic_raw);
                last_distance = Some(distance);
                in_range_samples += 1;
            }
            Ok(None) => {
                // 측정 범위를 벗어남
                out_of_range_samples += 1;
            }
            Err(e) => {
                // 측정 중 에러 발생
                eprintln!("[BSW] ultrasonic measurement error: {:?}", e);
            }
        }

        if last_log.elapsed() >= calibration.log_interval {
            let total = in_range_samples + out_of_range_samples;
            if total > 0 {
                if let Some(distance) = last_distance {
                    println!(
                        "[BSW] 초음파 요약: {}회 측정, 정상 {}회, 범위 초과 {}회, 최근 거리 {:.2}cm",
                        total, in_range_samples, out_of_range_samples, distance
                    );
                } else {
                    println!(
                        "[BSW] 초음파 요약: {}회 측정, 정상 {}회, 범위 초과 {}회, 최근 거리 없음",
                        total, in_range_samples, out_of_range_samples
                    );
                }
            }
            in_range_samples = 0;
            out_of_range_samples = 0;
            last_log = Instant::now();
        }

        // 증가하는 카운터는 수신 측에서 누락 여부를 감지하는 데 사용된다.
        alive_cnt += 1;
    }
}
