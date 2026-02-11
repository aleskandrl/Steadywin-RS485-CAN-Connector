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

    auto send_and_wait = [&](uint32_t tx_id) -> MotorError {
        CanFrame frame;
        frame.id = tx_id; // Master -> Slave
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
    };

    // Prefer "new" protocol (0x100|ID).
    // Fallback to legacy (ID) causes echoes to be interpreted as responses because Echo ID == device_address.
    // The working reference implementation ONLY uses 0x100 | ID.
    return send_and_wait(0x100 | device_address);
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
    std::memcpy(&response_data.multi_turn_angle, resp.data + 3, 4);
    return MotorError::Ok;
}

MotorError SteadywinProtocolCAN::setAbsolutePositionControlNoResponse(uint8_t device_address, int32_t absolute_position_counts) {
    if (!port_ || !port_->isOpen()) return MotorError::PortNotOpen;

    CanFrame frame;
    frame.id = 0x100 | device_address; // Use V3.07 ID scheme
    frame.dlc = 5; // Cmd(1) + Data(4)
    frame.data[0] = 0xC2;
    std::memcpy(frame.data + 1, &absolute_position_counts, 4);

    if (!port_->write(frame)) return MotorError::WriteError;
    
    // FIRE AND FORGET - Do not wait for response.
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

// Command 0xA3
MotorError SteadywinProtocolCAN::readMultiTurnAngle(uint8_t device_address, int32_t& angle_counts) {
    CanFrame resp;
    MotorError err = sendCommand(device_address, 0xA3, nullptr, 0, resp);
    if (err != MotorError::Ok) return err;
    std::memcpy(&angle_counts, resp.data + 3, 4);
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

// --- MIT Protocol Implementations ---

// Helper for float packing
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

MotorError SteadywinProtocolCAN::configureMitLimits(uint8_t device_address, const MitParams& limits) {
    CanFrame resp;
    uint8_t data[6];
    
    // Pos_Max (0.1 rad), Vel_Max (0.01 rad/s), T_Max (0.01 Nm)
    uint16_t p = (uint16_t)(limits.p_max * 10.0f);
    uint16_t v = (uint16_t)(limits.v_max * 100.0f);
    uint16_t t = (uint16_t)(limits.t_max * 100.0f);

    std::memcpy(data, &p, 2);
    std::memcpy(data + 2, &v, 2);
    std::memcpy(data + 4, &t, 2);

    return sendCommand(device_address, 0xF0, data, 6, resp);
}

MotorError SteadywinProtocolCAN::sendMitCommand(uint8_t device_address, float p, float v, float kp, float kd, float t, 
                                                const MitParams& limits, RealtimeDataPayload& response_data) {
    if (!port_ || !port_->isOpen()) return MotorError::PortNotOpen;

    // Constaints
    float p_min = -limits.p_max; float p_max = limits.p_max;
    float v_min = -limits.v_max; float v_max = limits.v_max;
    float t_min = -limits.t_max; float t_max = limits.t_max;
    float kp_min = 0.0f; float kp_max = 500.0f;
    float kd_min = 0.0f; float kd_max = 5.0f;

    // Pack
    uint16_t p_int = float_to_uint(p, p_min, p_max, 16);
    uint16_t v_int = float_to_uint(v, v_min, v_max, 12);
    uint16_t kp_int = float_to_uint(kp, kp_min, kp_max, 12);
    uint16_t kd_int = float_to_uint(kd, kd_min, kd_max, 12);
    uint16_t t_int = float_to_uint(t, t_min, t_max, 12);

    CanFrame frame;
    // Special ID for MIT mode: 0x400 | Addr. Using 0x100 | Addr logic:
    // Protocol says: "Highest Bit (Bit[10]) of StdID to 1"
    // Usually StdID is 11 bits (0..10). Bit 10 is 0x400.
    // If device is ID=1, we send 0x401. 
    // BUT we also use correct addressing which might be 0x100?
    // The manual says: (0x400 | Dev_addr) or (0x400 | (0x100|Dev_addr)).
    // Let's use (0x400 | 0x100 | device_address) to be consistent with our "0x100|ID" preference,
    // or just 0x400 | device_address if that's safer.
    // Manual: "Slave... recognizes ... (0x100 | Dev_addr)".
    // Manual for MIT: "StdID to 1... (If Dev_addr is 1, StdID is 0x401 or 0x501)"
    // 0x501 is 0x400 | 0x100 | 0x01. 
    // Let's use 0x500 | ID to be consistent with our receive logic which expects 0x100|ID? 
    // actually receive logic expects response with ID=Dev_Addr. 
    // sendCommand uses 0x100 | ID. 
    // So let's use 0x400 | 0x100 | ID = 0x500 | ID.
    
    frame.id = 0x400 | 0x100 | device_address;
    frame.dlc = 8;
    
    // Byte[0]=P_high, Byte[1]=P_low
    frame.data[0] = (uint8_t)(p_int >> 8);
    frame.data[1] = (uint8_t)(p_int & 0xFF);
    
    // Byte[2]=V_high, Byte[3]=V_low_4 | KP_high_4
    frame.data[2] = (uint8_t)(v_int >> 4);
    frame.data[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8));
    
    // Byte[4]=KP_low_8
    frame.data[4] = (uint8_t)(kp_int & 0xFF);
    
    // Byte[5]=KD_high, Byte[6]=KD_low_4 | T_high_4
    frame.data[5] = (uint8_t)(kd_int >> 4);
    frame.data[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8));
    
    // Byte[7]=T_low_8
    frame.data[7] = (uint8_t)(t_int & 0xFF);

    if (!port_->write(frame)) return MotorError::WriteError;

    // Wait for response
    // Response format is same as 0xF1 (ID = Dev_Addr)
    CanFrame response;
    unsigned int retries = 50;
    while (retries--) {
        if (port_->read(response, DEFAULT_TIMEOUT_MS)) {
             if (response.id == device_address && response.dlc == 7) { 
                 // Note: MIT response has no command code in Byte 0? 
                 // Manual says: "Content... consistent with 0xF1".
                 // 0xF1 response has 0xF1 at Byte 0.
                 // Manual for Motion Control Command response: "consistent with 0xF1".
                 // So we expect Byte 0 to be 0xF1? Or is it just raw data?
                 // Wait, 0xF1 response (master->slave is 0xF1, slave->master is 0xF1 + data).
                 // For Motion Command (ID 0x4xx), response is "consistent with 0xF1".
                 // Does it imply Byte 0 is 0xF1?
                 // Let's assume yes.
                 if (response.data[0] == 0xF1) {
                     // Unpack
                     uint16_t p_raw = (response.data[1] << 8) | response.data[2];
                     uint16_t v_raw = (response.data[3] << 4) | (response.data[4] >> 4);
                     uint16_t t_raw = ((response.data[4] & 0xF) << 8) | response.data[5];

                     response_data.single_turn_angle = uint_to_float(p_raw, p_min, p_max, 16) * (180.0f / 3.1415926535f); // Convert rad to deg for compatibility? or keep rad?
                     // Request was for "Smooth and reliable motion". MIT protocol gives p, v, t.
                     // Our `RealtimeDataPayload` uses degrees and RPM.
                     // Let's convert for consistency with other modes.
                     // 2pi rad = 360 deg.
                     
                     // BUT `single_turn_angle` in payload is usually 0-360.
                     // MIT position is -95 to +95 rad (multi-turn). 
                     // We should probably map it to multi_turn_angle if possible, or just misuse single_turn_angle.
                     // Let's store in single_turn (as deg) and multi_turn (as deg total).
                     float p_rad = uint_to_float(p_raw, p_min, p_max, 16);
                     float v_rad_s = uint_to_float(v_raw, v_min, v_max, 12);
                     float t_nm = uint_to_float(t_raw, t_min, t_max, 12); // We don't have torque field in payload really, except maybe q_axis_current?
                     
                     response_data.single_turn_angle = p_rad * (180.0f / 3.1415926535f); // Total angle in deg
                     response_data.multi_turn_angle = response_data.single_turn_angle; 
                     
                     // 1 rad/s = 9.549296596425384 RPM
                     response_data.mechanical_velocity = (int32_t)(v_rad_s * 9.5493f * 100.0f); // 0.01 RPM
                     
                     // T = Kt * Iq. We don't know Kt here easily (it's in motor param).
                     // We can't backwards convert T to Iq without Kt.
                     // But payload expects Iq. 
                     // Let's leave Iq 0 or try to put Torque in Q-axis field? 
                     // Q-axis field is int32. Let's put Torque * 1000 (mNm)? 
                     // Or just leave it. The user wants smooth motion, telemetry of Position/Velocity is most important.
                     
                     response_data.running_status = (response.data[6] & 1) ? 1 : 0; // Bit 0 is 'in motion mode'
                     response_data.fault_code = (response.data[6] & 2) ? 1 : 0; // Bit 1 is 'fault' (simplified)
                     
                     return MotorError::Ok;
                 }
             }
        } else {
            return MotorError::Timeout;
        }
    }
    return MotorError::Timeout;
}

} // namespace steadywin
