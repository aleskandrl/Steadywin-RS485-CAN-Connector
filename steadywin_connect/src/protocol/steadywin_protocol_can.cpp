#include "steadywin/protocol/steadywin_protocol_can.h"
#include <cstring>
#include <algorithm>
#include <vector>

// =============================================================================
// FINAL VERSION: RS485 COMMANDS + CORRECT CAN ADDRESSING
// =============================================================================

namespace steadywin {

SteadywinProtocolCAN::SteadywinProtocolCAN(std::shared_ptr<ICanPort> port)
    : port_(std::move(port)) {}

// Common send function with correct addressing and filtering
MotorError SteadywinProtocolCAN::sendCommand(uint8_t device_address, uint8_t command_code, const uint8_t* data, uint8_t len, CanFrame& response) {
    if (!port_ || !port_->isOpen()) return MotorError::PortNotOpen;

    CanFrame frame;
    frame.id = 0x100 | device_address; // Master -> Slave
    frame.dlc = 1 + len;
    frame.data[0] = command_code;
    if (data && len > 0) {
        std::memcpy(frame.data + 1, data, std::min<uint8_t>(len, 7));
    }

    if (!port_->write(frame)) return MotorError::WriteError;

    // Wait for response, skipping "junk" packets
    unsigned int retries = 50; 
    while (retries--) {
        if (port_->read(response, DEFAULT_TIMEOUT_MS)) {
            // Slave -> Master. Response ID = Motor ID
            if (response.id == device_address && response.data[0] == command_code) {
                return MotorError::Ok;
            }
        } else {
            return MotorError::Timeout;
        }
    }

    return MotorError::InvalidResponse;
}


// --- Concrete command implementations (codes same as RS485) ---

// Command 0x0B
MotorError SteadywinProtocolCAN::readRealtimeData(uint8_t device_address, RealtimeDataPayload& data) {
    CanFrame resp;
    // RS485 implementation of this command returns a packet the size of RealtimeDataPayload.
    // We assume CAN does the same but adds a response byte at the beginning.
    // Thus, data starts at resp.data[1].
    // But! CAN frame is only 8 bytes. RealtimeDataPayload is 26 bytes.
    // It is logical to assume that multiple CAN commands are used, as it was in v3.07.
    // Let's go back to the multiple request scheme but with CORRECT addressing.

    // VERSION 2: Use commands from CAN v3.07 but with correct sendCommand implementation
    MotorError err;

    // 1. Angles (command 0xA3)
    err = sendCommand(device_address, 0xA3, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    std::memcpy(&data.single_turn_angle, resp.data + 1, 2);
    std::memcpy(&data.multi_turn_angle, resp.data + 3, 4);

    // 2. Temperature, Current, Speed (command 0xA4)
    err = sendCommand(device_address, 0xA4, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    data.working_temperature = resp.data[1];
    int16_t cur_short, vel_short;
    std::memcpy(&cur_short, resp.data + 2, 2);
    std::memcpy(&vel_short, resp.data + 4, 2);
    data.q_axis_current = static_cast<int32_t>(cur_short);
    data.mechanical_velocity = static_cast<int32_t>(vel_short);

    // 3. Voltage, status, errors (command 0xAE)
    err = sendCommand(device_address, 0xAE, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    std::memcpy(&data.bus_voltage, resp.data + 1, 2);
    std::memcpy(&data.bus_current, resp.data + 3, 2);
    data.running_status = resp.data[6];
    data.fault_code = resp.data[7];
    data.motor_status = (data.running_status > 0) ? 1 : 0;

    return MotorError::Ok;
}

// Command 0xC2
MotorError SteadywinProtocolCAN::setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data) {
    CanFrame resp;
    uint8_t data[4];
    std::memcpy(data, &absolute_position_counts, 4);
    
    MotorError err = sendCommand(device_address, 0xC2, data, 4, resp);
    if (err != MotorError::Ok) return err;
    
    std::memcpy(&response_data.single_turn_angle, resp.data + 1, 2);
    std::memcpy(&response_data.multi_turn_angle, resp.data + 3, 4);
    return MotorError::Ok;
}

// Command 0xCF
MotorError SteadywinProtocolCAN::disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) {
    CanFrame resp;
    MotorError err = sendCommand(device_address, 0xCF, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    
    std::memcpy(&response_data.bus_voltage, resp.data + 1, 2);
    std::memcpy(&response_data.bus_current, resp.data + 3, 2);
    response_data.working_temperature = resp.data[5];
    response_data.running_status = resp.data[6];
    response_data.fault_code = resp.data[7];
    response_data.motor_status = 0;
    return MotorError::Ok;
}

// Command 0xAF
MotorError SteadywinProtocolCAN::clearFaults(uint8_t device_address, uint8_t& current_faults) {
    CanFrame resp;
    MotorError err = sendCommand(device_address, 0xAF, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    current_faults = resp.data[1];
    return MotorError::Ok;
}

// Command 0xB1
MotorError SteadywinProtocolCAN::setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) {
    CanFrame resp;
    MotorError err = sendCommand(device_address, 0xB1, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    std::memcpy(&mechanical_offset, resp.data + 1, 2);
    return MotorError::Ok;
}

// Command 0xC1
MotorError SteadywinProtocolCAN::setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) {
    CanFrame resp;
    
    uint8_t accel_data[4];
    std::memcpy(accel_data, &acceleration, 4);
    sendCommand(device_address, 0xB5, accel_data, 4, resp);
    
    uint8_t vel_data[4];
    std::memcpy(vel_data, &target_velocity_rpm_x100, 4);
    MotorError err = sendCommand(device_address, 0xC1, vel_data, 4, resp);
    if (err != MotorError::Ok) return err;
    
    int32_t current_vel;
    std::memcpy(&current_vel, resp.data + 1, 4);
    response_data.mechanical_velocity = current_vel;
    
    return MotorError::Ok;
}

// Command 0xC3
MotorError SteadywinProtocolCAN::setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) {
    CanFrame resp;
    uint8_t data[4];
    std::memcpy(data, &relative_counts, 4);
    
    MotorError err = sendCommand(device_address, 0xC3, data, 4, resp);
    if (err != MotorError::Ok) return err;
    
    std::memcpy(&response_data.single_turn_angle, resp.data + 1, 2);
    std::memcpy(&response_data.multi_turn_angle, resp.data + 3, 4);
    return MotorError::Ok;
}

// Command 0xCE
MotorError SteadywinProtocolCAN::setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) {
    CanFrame resp;
    MotorError err = sendCommand(device_address, 0xCE, &operation, 1, resp);
    if (err != MotorError::Ok) return err;
    status = resp.data[1];
    return MotorError::Ok;
}

// Bx commands for parameters
MotorError SteadywinProtocolCAN::readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) {
    CanFrame resp;
    if (sendCommand(device_address, 0xB6, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.pos_kp, resp.data + 1, 4);
    if (sendCommand(device_address, 0xB7, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.pos_ki, resp.data + 1, 4);
    if (sendCommand(device_address, 0xB8, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.vel_kp, resp.data + 1, 4);
    if (sendCommand(device_address, 0xB9, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.vel_ki, resp.data + 1, 4);
    if (sendCommand(device_address, 0xB2, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.pos_limit_rpm_x100, resp.data + 1, 4);
    if (sendCommand(device_address, 0xB3, nullptr, 0, resp) == MotorError::Ok) std::memcpy(&params.vel_limit_ma, resp.data + 1, 4);
    return MotorError::Ok;
}

MotorError SteadywinProtocolCAN::writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) {
    CanFrame resp;
    uint8_t data[4];
    std::memcpy(data, &params.pos_kp, 4); sendCommand(device_address, 0xB6, data, 4, resp);
    std::memcpy(data, &params.pos_ki, 4); sendCommand(device_address, 0xB7, data, 4, resp);
    std::memcpy(data, &params.vel_kp, 4); sendCommand(device_address, 0xB8, data, 4, resp);
    std::memcpy(data, &params.vel_ki, 4); sendCommand(device_address, 0xB9, data, 4, resp);
    std::memcpy(data, &params.pos_limit_rpm_x100, 4); sendCommand(device_address, 0xB2, data, 4, resp);
    std::memcpy(data, &params.vel_limit_ma, 4); sendCommand(device_address, 0xB3, data, 4, resp);
    return MotorError::Ok;
}

} // namespace steadywin
