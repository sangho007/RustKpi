use crate::bsw::lib::ultrasonic_lib::*;
use crate::rte::rte_dto::{DtoUltraSonicRaw, VfbEvent};
use crate::rte::rte_main::{DebugSender, VfbSender};
use hc_sr04::{HcSr04, Unit};
use std::sync::Arc;
use std::time::Duration;
use tokio::time;


pub async fn ea_ultrasonic_provider(tx: VfbSender, debug: DebugSender) {
    let mut alive_cnt = 0;
    let mut interval = time::interval(Duration::from_millis(100));

    // 센서 초기화는 실패하면 더 이상 진행할 수 없으므로, 에러를 출력하고 함수를 종료합니다.
    let mut sensor = match HcSr04::new(TRIGGER_PIN, ECHO_PIN, None) {
        Ok(s) => {
            println!("[BSW] 초음파 센서 초기화 성공");
            s // 성공하면 센서 객체를 반환
        },
        Err(e) => {
            eprintln!("[BSW] 초음파 센서 초기화 실패: {:?}. Provider를 종료합니다.", e);
            return; // 함수 종료
        }
    };

    let mut ultrasonic_raw;
    let mut event;

    loop {
        interval.tick().await;

        // 측정 실패는 일시적일 수 있으므로, 에러만 출력하고 루프는 계속 진행합니다.
        match sensor.measure_distance(Unit::Centimeters) {
            Ok(Some(distance)) => {
                // 측정 성공
                println!("{:.2} cm", distance);
                ultrasonic_raw = DtoUltraSonicRaw::new(distance, alive_cnt);
                event = VfbEvent::UltraSonicRawEvent(Arc::new(ultrasonic_raw));
                let _ = tx.send(event.clone());
                let _ = debug.send(event.clone());
            }
            Ok(None) => {
                // 측정 범위를 벗어남
                println!("[BSW] ultrasonic out of range");
            }
            Err(e) => {
                // 측정 중 에러 발생
                eprintln!("[BSW] ultrasonic measurement error: {:?}", e);
            }
        }

        alive_cnt += 1;
    }
}