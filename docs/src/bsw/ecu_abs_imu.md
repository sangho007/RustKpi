# ecu_abs_imu.rs — IMU 텔레메트리 파서

- 경로: `src/bsw/ecu_abs_imu.rs`
- 계층: BSW / ECU Abstraction (IMU)

## 목적
USB/TCP 게이트웨이에서 전달된 protobuf 페이로드를 파싱해 ASW에서 사용 가능한 `DtoImu`로 변환합니다. IMU 헤더/자세/속도/가속도/자이로 정보를 모두 옵션 필드로 보존해 후속 제어·로깅에 활용할 수 있습니다.

## 처리 흐름
1. `ImuChannels.raw_tx.subscribe()`로 `DtoTcpTelemetry` 스트림을 구독합니다.
2. 새 페이로드가 도착할 때마다 `imu_proto::decode_imu(payload, alive_cnt)`를 호출해 protobuf를 디코딩합니다.
3. 성공 시 `DtoImu`를 `Arc`로 감싸 `ImuChannels.parsed_tx`에 브로드캐스트합니다.
4. 디코딩 에러는 alive 카운터와 함께 경고로 기록하며, 채널 지연(`RecvError::Lagged`)도 모니터링합니다.
5. 송신자가 종료되면(`RecvError::Closed`) 루프를 빠져나와 태스크를 종료합니다.

## 연관 모듈
- `bsw::lib::imu_proto`: prost 기반 protobuf 스키마와 헬퍼를 포함하며, 쿼터니언 → yaw/roll/pitch 계산을 수행합니다.
- `rte::rte_dto::DtoImu*`: IMU DTO 구조체를 정의합니다.
