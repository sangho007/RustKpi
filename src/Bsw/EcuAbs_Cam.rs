use std::sync::Arc;
use crate::Rte::Rte_Main::{VfbSender, DebugSender};
use crate::Rte::Rte_Dto::{VfbEvent, Dto_CamRaw};
use std::time::Duration;
use opencv::core::Mat;
use opencv::{videoio, Result, prelude::*};
use tokio::time;

const CAM_MODE: i32 = 1;

pub async fn EA_CamProvider(tx: VfbSender, debug: DebugSender) -> Result<()> {
    let mut alive_cnt = 0;

    // 비디오 파일이나 카메라 장치를 엽니다.
    let cammode = false;
    let mut cap = if cammode {
        videoio::VideoCapture::new(0, videoio::CAP_ANY)?
    } else {
        videoio::VideoCapture::from_file("./video/challenge.mp4", videoio::CAP_ANY)?
    };
    let mut interval = time::interval(Duration::from_millis(33));


    loop {
        interval.tick().await;
        let mut frame = Mat::default();

        match cap.read(&mut frame) {
            Ok(is_read) => {
                // 프레임을 못 읽어오거나 비어있으면 루프를 종료합니다.
                if !is_read || frame.empty() {
                    println!("[bsw] 비디오 스트림 종료.");
                    break;
                }
            }
            Err(e) => {
                eprintln!("[bsw] 에러: 프레임 읽기 실패: {:?}", e);
                break;
            }
        }

        let cam_raw = Dto_CamRaw::new(Arc::new(frame),1280, 720, alive_cnt);

        // 새로 만든 구조체의 소유권을 Arc로 넘깁니다.
        let event = VfbEvent::CamRawData(Arc::new(cam_raw));

        let _ = tx.send(event.clone());

        let _ = debug.send(event.clone());

        alive_cnt += 1;
    }

    Ok(())
}