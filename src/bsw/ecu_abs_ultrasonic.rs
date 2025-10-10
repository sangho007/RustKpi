use std::sync::Arc;
use crate::rte::rte_main::{VfbSender, DebugSender};
use crate::rte::rte_dto::{VfbEvent, DtoCamRaw};
use std::time::Duration;
use hc_sr04::{HcSr04, Unit};
use tokio::time;

const TRIGGER_PIN: u8 = 23;
const ECHO_PIN: u8 = 24;

pub async fn ea_ultrasonic_provider(tx: VfbSender, debug: DebugSender) -> Result<(), Box<dyn std::error::Error>>{
    let mut alive_cnt = 0;
    let mut interval = time::interval(Duration::from_millis(100));

    let mut sensor = HcSr04::new(TRIGGER_PIN, ECHO_PIN, None)?;

    loop {
        interval.tick().await;

        match sensor.measure_distance(Unit::Centimeters)? {
            Some(distance) => {
                // 측정된 거리를 소수점 둘째 자리까지 형식에 맞춰 출력합니다.
                println!("{:.2} cm", distance);
            }
            None => {
                // 센서의 측정 범위를 벗어났을 경우 메시지를 출력합니다.
                // Python gpiozero의 max_distance와 유사한 기능입니다.
                println!("[BSW] ultrasonic error");
            }
        }

        alive_cnt += 1;
    }
}