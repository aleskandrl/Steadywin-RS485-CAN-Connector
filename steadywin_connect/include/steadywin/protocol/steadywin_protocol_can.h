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
    MotorError setAbsolutePositionControlNoResponse(uint8_t device_address, int32_t absolute_position_counts) override;
    MotorError disableMotor(uint8_t device_address, RealtimeDataPayload& response_data) override;
    MotorError clearFaults(uint8_t device_address, uint8_t& current_faults) override;
    MotorError readMultiTurnAngle(uint8_t device_address, int32_t& angle_counts) override;
    MotorError setZeroPoint(uint8_t device_address, uint16_t& mechanical_offset) override;
    MotorError setVelocityControl(uint8_t device_address, int32_t target_velocity_rpm_x100, uint32_t acceleration, RealtimeDataPayload& response_data) override;
    MotorError setRelativePositionControl(uint8_t device_address, int32_t relative_counts, RealtimeDataPayload& response_data) override;
    MotorError setBrakeControl(uint8_t device_address, uint8_t operation, uint8_t& status) override;
    MotorError readMotionControlParameters(uint8_t device_address, MotionControlParametersPayload& params) override;
    MotorError writeMotionControlParameters(uint8_t device_address, const MotionControlParametersPayload& params) override;


    /**
     * @brief Parameters for MIT Motion Control Mode limits.
     * All units are as per protocol (Rad, Rad/s, Nm).
     */
    struct MitParams {
        float p_max{95.5f};   // Max Position (rad)
        float v_max{45.0f};   // Max Velocity (rad/s)
        float t_max{18.0f};   // Max Torque (Nm)
    };

    /**
     * @brief Configures the limits for MIT Motion Control Mode (0xF0).
     * @param device_address The address of the motor.
     * @param limits The limits to set.
     * @return MotorError::Ok on success.
     */
    MotorError configureMitLimits(uint8_t device_address, const MitParams& limits);

    /**
     * @brief Sends a Motion Control Command (MIT Mode).
     * This command puts the motor into Motion Control Mode immediately.
     * To exit, send disableMotor (0xCF) or setZeroPoint (0xB1).
     * 
     * @param device_address The address of the motor.
     * @param p Target Position (rad)
     * @param v Target Velocity (rad/s)
     * @param kp Position Gain (0-500)
     * @param kd Velocity Gain (0-5)
     * @param t Feedforward Torque (Nm)
     * @param limits Current limits used for packing the float values.
     * @param response_data Output for the received status.
     * @return MotorError::Ok on success.
     */
    MotorError sendMitCommand(uint8_t device_address, float p, float v, float kp, float kd, float t, 
                              const MitParams& limits, RealtimeDataPayload& response_data);

private:
    /**
     * @brief Sends a CAN command and optionally waits for a response.
     */
    MotorError sendCommand(uint8_t device_address, uint8_t command_code, const uint8_t* data, uint8_t len, CanFrame& response);

    std::shared_ptr<ICanPort> port_;
    
    // Hot-path CAN command timeout.
    // 100ms is too high for control streaming and caps effective TX rate.
    static constexpr unsigned int DEFAULT_TIMEOUT_MS = 8;
};

} // namespace steadywin
