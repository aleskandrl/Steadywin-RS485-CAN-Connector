#pragma once

#include "steadywin/protocol/steadywin_protocol.h"
#include "steadywin/hal/can_port_interface.h"
#include <memory>

namespace steadywin {

/**
 * @class SteadywinProtocolCAN
 * @brief Implements the Custom CAN Communication Protocol V3.07b0 for Steadywin motors.
 */
class SteadywinProtocolCAN : public SteadywinProtocol {
public:
    /**
     * @brief Constructor.
     * @param port A shared pointer to an ICanPort implementation.
     */
    explicit SteadywinProtocolCAN(std::shared_ptr<ICanPort> port);

    MotorError readRealtimeData(uint8_t device_address, RealtimeDataPayload& data) override;
    MotorError setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data) override;
    MotorError disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) override;
    MotorError clearFaults(uint8_t device_address, uint8_t& current_faults) override;
    MotorError setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) override;
    MotorError setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) override;
    MotorError setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) override;
    MotorError setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) override;
    MotorError readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) override;
    MotorError writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) override;

private:
    /**
     * @brief Sends a CAN command and optionally waits for a response.
     */
    MotorError sendCommand(uint8_t device_address, uint8_t command_code, const uint8_t* data, uint8_t len, CanFrame& response);

    std::shared_ptr<ICanPort> port_;
    
    static constexpr unsigned int DEFAULT_TIMEOUT_MS = 100;
};

} // namespace steadywin
