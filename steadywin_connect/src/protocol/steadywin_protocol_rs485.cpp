#include "steadywin/steadywin_protocol_rs485.h"
#include <cstring> // For memcpy

namespace steadywin {

// Protocol constants
constexpr uint8_t MASTER_HEADER = 0xAE;
constexpr uint8_t SLAVE_HEADER = 0xAC;
constexpr size_t MIN_RESPONSE_SIZE = 7; // Header(1)+Seq(1)+Addr(1)+Cmd(1)+Len(1)+CRC(2)
constexpr size_t HEADER_TO_DATA_LEN = 5; // Header(1)+Seq(1)+Addr(1)+Cmd(1)+Len(1)

SteadywinProtocolRS485::SteadywinProtocolRS485(std::shared_ptr<ISerialPort> port)
    : port_(std::move(port)) {}

MotorError SteadywinProtocolRS485::readRealtimeData(uint8_t device_address, RealtimeDataPayload& data) {
    std::vector<uint8_t> response_payload;
    // Command 0x0B has an empty request payload
    MotorError result = sendAndReceive(device_address, 0x0B, {}, response_payload);
    if (result != MotorError::Ok) {
        return result;
    }

    // Validate that the response payload has the expected size
    if (response_payload.size() != sizeof(RealtimeDataPayload)) {
        return MotorError::InvalidResponse;
    }

    // Copy the payload into the output struct
    std::memcpy(&data, response_payload.data(), sizeof(RealtimeDataPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::clearFaults(uint8_t device_address, uint8_t& current_faults) {
    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x0F, {}, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != 1) return MotorError::InvalidResponse;
    current_faults = response_payload[0];
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) {
    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x1D, {}, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != 2) return MotorError::InvalidResponse;
    std::memcpy(&mechanical_offset, response_payload.data(), 2);
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) {
    std::vector<uint8_t> request_payload(8);
    std::memcpy(request_payload.data(), &target_velocity_rpm_x100, 4);
    std::memcpy(request_payload.data() + 4, &acceleration, 4);

    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x21, request_payload, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != sizeof(RealtimeDataPayload)) return MotorError::InvalidResponse;
    std::memcpy(&response_data, response_payload.data(), sizeof(RealtimeDataPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) {
    std::vector<uint8_t> request_payload(4);
    std::memcpy(request_payload.data(), &relative_counts, 4);

    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x23, request_payload, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != sizeof(RealtimeDataPayload)) return MotorError::InvalidResponse;
    std::memcpy(&response_data, response_payload.data(), sizeof(RealtimeDataPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) {
    std::vector<uint8_t> request_payload = { operation };
    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x2E, request_payload, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != 1) return MotorError::InvalidResponse;
    status = response_payload[0];
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) {
    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x14, {}, response_payload);
    if (result != MotorError::Ok) return result;
    if (response_payload.size() != sizeof(MotionControlParametersPayload)) return MotorError::InvalidResponse;
    std::memcpy(&params, response_payload.data(), sizeof(MotionControlParametersPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) {
    std::vector<uint8_t> request_payload(sizeof(MotionControlParametersPayload));
    std::memcpy(request_payload.data(), &params, sizeof(MotionControlParametersPayload));
    std::vector<uint8_t> response_payload;
    // 0x15: Write but do not save (to RAM)
    MotorError result = sendAndReceive(device_address, 0x15, request_payload, response_payload);
    if (result != MotorError::Ok) return result;
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data) {
    std::vector<uint8_t> request_payload(4);
    // Pack the position value in Little-Endian format
    std::memcpy(request_payload.data(), &absolute_position_counts, sizeof(absolute_position_counts));

    std::vector<uint8_t> response_payload;
    MotorError result = sendAndReceive(device_address, 0x22, request_payload, response_payload);
    if (result != MotorError::Ok) {
        return result;
    }

    // This command returns a standard telemetry packet
    if (response_payload.size() != sizeof(RealtimeDataPayload)) {
        return MotorError::InvalidResponse;
    }

    std::memcpy(&response_data, response_payload.data(), sizeof(RealtimeDataPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) {
    std::vector<uint8_t> response_payload;
    // Command 0x2F has an empty request payload
    MotorError result = sendAndReceive(device_address, 0x2F, {}, response_payload);
    if (result != MotorError::Ok) {
        return result;
    }

    // This command returns a standard telemetry packet
    if (response_payload.size() != sizeof(RealtimeDataPayload)) {
        return MotorError::InvalidResponse;
    }

    std::memcpy(&response_data, response_payload.data(), sizeof(RealtimeDataPayload));
    return MotorError::Ok;
}

MotorError SteadywinProtocolRS485::sendAndReceive(uint8_t device_address, uint8_t command_code, const std::vector<uint8_t>& request_payload, std::vector<uint8_t>& response_payload) {
    if (!port_ || !port_->isOpen()) {
        return MotorError::PortNotOpen;
    }

    // 1. Build the request packet
    std::vector<uint8_t> request_packet;
    request_packet.reserve(HEADER_TO_DATA_LEN + request_payload.size() + 2); // 2 for CRC
    request_packet.push_back(MASTER_HEADER);
    request_packet.push_back(packet_sequence_);
    request_packet.push_back(device_address);
    request_packet.push_back(command_code);
    request_packet.push_back(static_cast<uint8_t>(request_payload.size()));
    if (!request_payload.empty()) {
        request_packet.insert(request_packet.end(), request_payload.begin(), request_payload.end());
    }

    uint16_t crc = calculateCrc16Modbus(request_packet.data(), request_packet.size());
    request_packet.push_back(static_cast<uint8_t>(crc & 0xFF)); // CRC Low Byte
    request_packet.push_back(static_cast<uint8_t>(crc >> 8));  // CRC High Byte

    // 2. Send the packet
    if (port_->write(request_packet) != static_cast<long long>(request_packet.size())) {
        return MotorError::WriteError;
    }

    // 3. Receive the response
    std::vector<uint8_t> response_buffer;
    if (port_->read(response_buffer, DEFAULT_TIMEOUT_MS) <= 0) {
        return MotorError::Timeout;
    }

    // 4. Validate the response packet
    if (response_buffer.size() < MIN_RESPONSE_SIZE) {
        return MotorError::InvalidResponse;
    }
    if (response_buffer[0] != SLAVE_HEADER) {
        return MotorError::InvalidResponse;
    }
    if (response_buffer[1] != packet_sequence_) {
        return MotorError::PacketSequenceMismatch;
    }
    // We can add more checks like address and command code if needed

    uint8_t response_len = response_buffer[4];
    if (response_buffer.size() != (HEADER_TO_DATA_LEN + response_len + 2)) {
        return MotorError::InvalidResponse; // Size mismatch
    }

    uint16_t received_crc = (static_cast<uint16_t>(response_buffer.back()) << 8) | response_buffer[response_buffer.size() - 2];
    uint16_t calculated_crc = calculateCrc16Modbus(response_buffer.data(), response_buffer.size() - 2);

    if (received_crc != calculated_crc) {
        return MotorError::CrcMismatch;
    }

    // 5. Extract the payload
    if (response_len > 0) {
        response_payload.assign(response_buffer.begin() + HEADER_TO_DATA_LEN, response_buffer.begin() + HEADER_TO_DATA_LEN + response_len);
    } else {
        response_payload.clear();
    }
    
    // Increment sequence for the next packet
    packet_sequence_++;

    return MotorError::Ok;
}

uint16_t SteadywinProtocolRS485::calculateCrc16Modbus(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

} // namespace steadywin
