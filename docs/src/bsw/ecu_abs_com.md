

## 아이폰 송신용 통신 설정 요약

- **전송 프로토콜**: TCP (usbmuxd/iproxy를 통해 `localhost`에서 수신자와 터널링)
- **호스트 주소**: `127.0.0.1`  
  - 실 기기 ↔ 맥 간 USB 연결 후 `iproxy 4820 4820` 실행을 가정
- **포트**: `4820`

## 페이로드 포맷

- **전송 주기**: 최소 30 Hz  
  - 앱 내부 타이머가 1/30초마다 DTO를 인코딩
- **프레이밍**: 길이 + 본문
  1. 4바이트 `uint32` 길이 헤더 (전송할 프로토버퍼 페이로드의 바이트 수)
  2. 길이만큼의 프로토버퍼 인코딩 페이로드
  - **엔디언**: 길이 헤더는 Big-Endian (`UInt32.bigEndian`)

- **메시지 직렬화**: 커스텀 프로토콜 버퍼 인코더  
  - `TelemetryProtobufEncoder`가 JSON 대신 DTO 구조를 protobuf wire format으로 변환
  - 메시지 구조는 `schema_version`, `header`, `status`, `pose_world_phone` 등 DTO 필드와 1:1 매핑

## 연결 및 재시도 규칙

- 앱에서 “수집 시작” 버튼 누를 때 연결 시도
- 실패 시 최대 5회까지 재시도 (기본 1.5초 간격)
- 연결이 준비되면 센서 수집과 전송을 동시에 시작
- 전송 중 에러 발생 시 연결 종료 후 재시도 루프 재개

## 기타 주의 사항

- 시뮬레이터에서는 ARKit/CoreMotion/네트워크 흐름이 모두 작동하지 않으므로 실 기기에서만 확인
- 수신 측에서도 4바이트 길이 헤더를 Big-Endian으로 디코딩한 뒤 protobuf 파서를 적용해야 함




사용한 protobuf swift code

```swift
import Foundation

/// 간단한 프로토콜 버퍼 인코더 구현 (SwiftProtobuf 없이 사용)
enum TelemetryProtobufEncoder {
    static func encode(_ dto: TelemetryDTO) -> Data {
        var writer = ProtoWriter()

        writer.writeString(fieldNumber: 1, value: dto.schemaVersion)
        writer.writeMessage(fieldNumber: 2) { header in
            header.writeVarint(fieldNumber: 1, value: UInt64(dto.header.stampNS))
            header.writeVarint(fieldNumber: 2, value: UInt64(dto.header.dtNS))
            header.writeVarint(fieldNumber: 3, value: UInt64(dto.header.seq))
            header.writeString(fieldNumber: 4, value: dto.header.sessionID)
            header.writeString(fieldNumber: 5, value: dto.header.clockDomain)
            header.writeString(fieldNumber: 6, value: dto.header.frameID)
            header.writeString(fieldNumber: 7, value: dto.header.childFrameID)
        }

        writer.writeMessage(fieldNumber: 3) { status in
            status.writeString(fieldNumber: 1, value: dto.status.tracking)
            status.writeDouble(fieldNumber: 2, value: dto.status.trackingConfidence)
            status.writeVarint(fieldNumber: 3, value: UInt64(dto.status.numFeatures))
            status.writeString(fieldNumber: 4, value: dto.status.statusReason)
            dto.status.flags.forEach { status.writeString(fieldNumber: 5, value: $0) }
        }

        writer.writeMessage(fieldNumber: 4) { pose in
            pose.writeMessage(fieldNumber: 1) { position in
                position.writeVector3(dto.poseWorldPhone.position)
            }
            pose.writeMessage(fieldNumber: 2) { orientation in
                orientation.writeQuaternion(dto.poseWorldPhone.orientationQuat)
            }
            pose.writeMessage(fieldNumber: 3) { cov in
                cov.writePackedDoubles(fieldNumber: 1, values: dto.poseWorldPhone.cov.pos)
                cov.writePackedDoubles(fieldNumber: 2, values: dto.poseWorldPhone.cov.ori)
            }
            pose.writeBool(fieldNumber: 4, value: dto.poseWorldPhone.valid)
        }

        writer.writeMessage(fieldNumber: 5) { velocity in
            velocity.writeMessage(fieldNumber: 1) { world in
                world.writeVector3(dto.velocity.world)
            }
            velocity.writeString(fieldNumber: 2, value: dto.velocity.source)
            velocity.writePackedDoubles(fieldNumber: 3, values: dto.velocity.cov)
            velocity.writeBool(fieldNumber: 4, value: dto.velocity.valid)
        }

        writer.writeMessage(fieldNumber: 6) { acceleration in
            acceleration.writeMessage(fieldNumber: 1) { body in
                body.writeVector3(dto.acceleration.bodyNoGravity)
            }
            acceleration.writeMessage(fieldNumber: 2) { world in
                world.writeVector3(dto.acceleration.world)
            }
            acceleration.writeString(fieldNumber: 3, value: dto.acceleration.source)
            acceleration.writePackedDoubles(fieldNumber: 4, values: dto.acceleration.cov)
            acceleration.writeBool(fieldNumber: 5, value: dto.acceleration.valid)
        }

        writer.writeMessage(fieldNumber: 7) { gyro in
            gyro.writeMessage(fieldNumber: 1) { body in
                body.writeVector3(dto.gyro.body)
            }
            gyro.writeString(fieldNumber: 2, value: dto.gyro.source)
            gyro.writeMessage(fieldNumber: 3) { bias in
                bias.writeVector3(dto.gyro.bias)
            }
            gyro.writePackedDoubles(fieldNumber: 4, values: dto.gyro.cov)
            gyro.writeBool(fieldNumber: 5, value: dto.gyro.valid)
        }

        writer.writeMessage(fieldNumber: 8) { calib in
            calib.writeMessage(fieldNumber: 1) { transform in
                transform.writePackedDoubles(fieldNumber: 1, values: dto.calib.tPhoneCar.rRowMajor)
                transform.writePackedDoubles(fieldNumber: 2, values: dto.calib.tPhoneCar.t)
            }
            calib.writeString(fieldNumber: 2, value: dto.calib.worldAlignment)
            calib.writeMessage(fieldNumber: 3) { detail in
                detail.writeBool(fieldNumber: 1, value: dto.calib.worldAlignmentDetail.yUp)
                detail.writeBool(fieldNumber: 2, value: dto.calib.worldAlignmentDetail.zForward)
            }
        }

        writer.writeMessage(fieldNumber: 9) { origin in
            origin.writeVarint(fieldNumber: 1, value: UInt64(dto.originReset.originID))
            origin.writeVarint(fieldNumber: 2, value: UInt64(dto.originReset.applyAtStampNS))
            origin.writeString(fieldNumber: 3, value: dto.originReset.nonce)
            origin.writeString(fieldNumber: 4, value: dto.originReset.reason)
        }

        writer.writeMessage(fieldNumber: 10) { integrity in
            integrity.writeString(fieldNumber: 1, value: dto.integrity.crc32)
        }

        return writer.data
    }
}

// MARK: - ProtoWriter

private enum WireType: UInt8 {
    case varint = 0
    case fixed64 = 1
    case lengthDelimited = 2
    case fixed32 = 5
}

private struct ProtoWriter {
    private(set) var data = Data()

    mutating func writeVarint(fieldNumber: Int, value: UInt64) {
        guard fieldNumber > 0 else { return }
        writeKey(fieldNumber: fieldNumber, wireType: .varint)
        writeRawVarint(value)
    }

    mutating func writeBool(fieldNumber: Int, value: Bool) {
        writeVarint(fieldNumber: fieldNumber, value: value ? 1 : 0)
    }

    mutating func writeDouble(fieldNumber: Int, value: Double) {
        guard fieldNumber > 0 else { return }
        writeKey(fieldNumber: fieldNumber, wireType: .fixed64)
        var bits = value.bitPattern.littleEndian
        withUnsafeBytes(of: &bits) { data.append(contentsOf: $0) }
    }

    mutating func writeString(fieldNumber: Int, value: String) {
        guard let bytes = value.data(using: .utf8) else { return }
        writeLengthDelimited(fieldNumber: fieldNumber, data: bytes)
    }

    mutating func writeLengthDelimited(fieldNumber: Int, data nested: Data) {
        guard fieldNumber > 0 else { return }
        writeKey(fieldNumber: fieldNumber, wireType: .lengthDelimited)
        writeRawVarint(UInt64(nested.count))
        data.append(nested)
    }

    mutating func writePackedDoubles(fieldNumber: Int, values: [Double]) {
        guard !values.isEmpty else { return }
        var packed = Data()
        packed.reserveCapacity(values.count * MemoryLayout<Double>.size)
        for value in values {
            var bits = value.bitPattern.littleEndian
            withUnsafeBytes(of: &bits) { packed.append(contentsOf: $0) }
        }
        writeLengthDelimited(fieldNumber: fieldNumber, data: packed)
    }

    mutating func writeMessage(fieldNumber: Int, build: (inout ProtoWriter) -> Void) {
        var nested = ProtoWriter()
        build(&nested)
        writeLengthDelimited(fieldNumber: fieldNumber, data: nested.data)
    }

    mutating func writeVector3(_ vector: TelemetryDTO.Vector3) {
        writeDouble(fieldNumber: 1, value: vector.x)
        writeDouble(fieldNumber: 2, value: vector.y)
        writeDouble(fieldNumber: 3, value: vector.z)
    }

    mutating func writeQuaternion(_ quat: TelemetryDTO.Quaternion) {
        writeDouble(fieldNumber: 1, value: quat.x)
        writeDouble(fieldNumber: 2, value: quat.y)
        writeDouble(fieldNumber: 3, value: quat.z)
        writeDouble(fieldNumber: 4, value: quat.w)
    }

    private mutating func writeKey(fieldNumber: Int, wireType: WireType) {
        let key = UInt64(fieldNumber << 3) | UInt64(wireType.rawValue)
        writeRawVarint(key)
    }

    private mutating func writeRawVarint(_ value: UInt64) {
        var raw = value
        while true {
            let byte = UInt8(raw & 0x7F)
            raw >>= 7
            if raw == 0 {
                data.append(byte)
                break
            } else {
                data.append(byte | 0x80)
            }
        }
    }
}
```

