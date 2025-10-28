# imu_proto.rs — IMU 프로토버퍼 디코더

- 경로: `src/bsw/lib/imu_proto.rs`
- 계층: BSW / Library

## 목적
iOS 기반 IMU 텔레메트리 프로토버퍼를 디코드해 RTE DTO (`DtoImu`) 구조체로 변환하며, 누락된 필드는 기본값으로 보완합니다. 또한 쿼터니언 자세를 yaw/roll/pitch 오일러 각으로 변환해 상위 제어 로직이 직접 사용할 수 있도록 합니다.

## 주요 기능
- `decode_imu`: 프로토 패킷을 파싱해 헤더, 상태, 포즈, 속도, 가속도, 자이로 데이터를 DTO로 매핑하고 `alive_cnt`를 그대로 유지합니다.
- `quaternion_to_yaw_roll_pitch`: 차량 좌표계에 맞게 쿼터니언을 오일러 각으로 변환합니다.
- 보조 컨버터(`to_header`, `to_status`, ...)가 각 서브 메시지를 안전하게 다루며, 문자열 필드는 노멀라이즈(빈 문자열 처리) 합니다.

## 테스트
- 단위 테스트가 quaterion→오일러 변환이 기대한 각도를 출력하는지 검증합니다.

