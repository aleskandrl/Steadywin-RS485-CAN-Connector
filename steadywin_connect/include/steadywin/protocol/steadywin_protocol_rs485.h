#pragma once

#include "steadywin/protocol/steadywin_protocol.h"
#include "steadywin/hal/serial_port_interface.h"
#include <memory>

namespace steadywin {

/**
 * @class SteadywinProtocolRS485
 * @brief Implements the low-level RS485 communication protocol for Steadywin motors.
 */
class SteadywinProtocolRS485 : public SteadywinProtocol {
public:
    /**
     * @brief Constructor.
     * @param port A shared pointer to an ISerialPort implementation.
     */
    explicit SteadywinProtocolRS485(std::shared_ptr<ISerialPort> port);

    MotorError readRealtimeData(uint8_t device_address, RealtimeDataPayload& data) override;
    MotorError setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data) override;
    MotorError disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) override;
    MotorError clearFaults(uint8_t device_address, uint8_t& current_faults) override;
    MotorError readMultiTurnAngle(uint8_t device_address, int32_t& angle_counts) override;
    MotorError setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) override;
    MotorError setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) override;
    MotorError setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) override;
    MotorError setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) override;
    MotorError readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) override;
    MotorError writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) override;

private:
    /**
     * @brief The core function to send a command and wait for a response.
     */
    MotorError sendAndReceive(uint8_t device_address, uint8_t command_code, const std::vector<uint8_t>& request_payload, std::vector<uint8_t>& response_payload);

    /**
     * @brief Calculates the CRC16-MODBUS checksum for a given data buffer.
     */
    static uint16_t calculateCrc16Modbus(const uint8_t* data, size_t length);

    std::shared_ptr<ISerialPort> port_;
    uint8_t packet_sequence_{0};
    
    static constexpr unsigned int DEFAULT_TIMEOUT_MS = 100;
};

} // namespace steadywin
