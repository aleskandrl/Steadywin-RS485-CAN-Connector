#pragma once

#include "steadywin/protocol/motor_error.h"
#include "steadywin/protocol/steadywin_types.h"
#include <cstdint>
#include <vector>

namespace steadywin {

/**
 * @class SteadywinProtocol
 * @brief Abstract base class for Steadywin communication protocols (RS485, CAN, etc.).
 * This interface defines the high-level commands that can be sent to a motor.
 */
class SteadywinProtocol {
public:
    virtual ~SteadywinProtocol() = default;

    /**
     * @brief Reads real-time data (angle, velocity, current, etc.).
     * @param device_address The address of the slave device.
     * @param[out] data A struct to be filled with the raw telemetry payload.
     * @return MotorError::Ok on success.
     */
    virtual MotorError readRealtimeData(uint8_t device_address, RealtimeDataPayload& data) = 0;

    /**
     * @brief Sets absolute position control mode.
     * @param device_address The address of the slave device.
     * @param absolute_position_counts Target position in encoder counts.
     * @param[out] response_data The telemetry data received in the response.
     * @return MotorError::Ok on success.
     */
    virtual MotorError setAbsolutePositionControl(uint8_t device_address, int32_t absolute_position_counts, RealtimeDataPayload& response_data) = 0;

    /**
     * @brief Disables the motor output (freewheel).
     * @param device_address The address of the slave device.
     * @param[out] response_data The telemetry data received in the response.
     * @return MotorError::Ok on success.
     */
    virtual MotorError disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) = 0;

    /**
     * @brief [0x0F] Clear faults.
     */
    virtual MotorError clearFaults(uint8_t device_address, uint8_t& current_faults) = 0;

    /**
     * @brief [0xA3] Read multi-turn angle from the motor.
     */
    virtual MotorError readMultiTurnAngle(uint8_t device_address, int32_t& angle_counts) = 0;

    /**
     * @brief Sets current position as zero point.
     */
    virtual MotorError setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) = 0;

    /**
     * @brief Velocity control.
     */
    virtual MotorError setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) = 0;

    /**
     * @brief Relative position control.
     */
    virtual MotorError setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) = 0;

    /**
     * @brief Holding brake control.
     * @param operation 0x00: Open, 0x01: Closed, 0xFF: Read
     */
    virtual MotorError setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) = 0;

    /**
     * @brief Read motion control parameters.
     */
    virtual MotorError readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) = 0;

    /**
     * @brief Write motion control parameters.
     */
    virtual MotorError writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) = 0;
};

} // namespace steadywin
