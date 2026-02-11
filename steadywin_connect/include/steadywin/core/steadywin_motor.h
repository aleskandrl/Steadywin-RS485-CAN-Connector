#pragma once

#include "steadywin/protocol/motor_error.h"
#include "steadywin/protocol/steadywin_protocol.h"
#include "steadywin/protocol/steadywin_protocol_can.h"
#include "steadywin/protocol/steadywin_types.h"
#include <cstdint>
#include <memory>
#include <mutex>

namespace steadywin {

// Forward declaration to break circular dependency, not strictly needed but good practice
class MotorManager;

/**
 * @class SteadywinMotor
 * @brief High-level controller for a single Steadywin servo motor.
 * This class provides a user-friendly API for controlling the motor, handling
 * unit conversions, and managing state. It implements the "czar-methods"
 * for common robotics tasks. 
 * NOTE: This class is now intended to be created and managed by the MotorManager.
 */
class SteadywinMotor {
public:
    /**
     * @brief Constructor. Intended to be called by MotorManager.
     * @param device_address The bus address of the motor (1-254).
     * @param protocol A shared pointer to a SteadywinProtocol implementation (owned by MotorManager).
     * @param bus_mutex A reference to the global bus mutex (owned by MotorManager).
     */
    SteadywinMotor(uint8_t device_address, std::shared_ptr<SteadywinProtocol> protocol, std::recursive_mutex& bus_mutex);

    /**
     * @brief Drains any pending bytes/frames from the underlying transport buffer.
     * Useful during startup/shutdown safety handshakes to avoid stale replies.
     */
    void drainBuffer();

    /**
     * @brief Initializes communication with the motor.
     * Must be called before any other control methods. This method communicates
     * with the device to verify its presence.
     * @return MotorError::Ok on success.
     */
    MotorError initialize();

    // --- High-Level "Czar" Methods ---

    /**
     * @brief Disables the motor (freewheeling).
     * The motor shaft will be free to rotate. This is the default state after power-on.
     * @return MotorError::Ok on success.
     */
    MotorError disable();
    
    /**
     * @brief Commands the motor to hold its current position.
     * This is a simple way to enable the motor. The motor will resist external forces.
     * @return MotorError::Ok on success.
     */
    MotorError holdPosition();

    /**
     * @brief Commands the motor to move to an absolute multi-turn position.
     * This is a non-blocking call. Use getTelemetry() to monitor progress.
     * @param angle_degrees The target absolute angle in degrees.
     * @return MotorError::Ok on success, or an error code on failure.
     */
    MotorError moveTo(double angle_degrees);

    /**
     * @brief Sends a target position command WITHOUT waiting for a response/confirmation.
     * Extremely fast operation (just writes to bus buffer).
     * @param angle_degrees Target absolute angle in degrees.
     * @return MotorError::Ok on success.
     */
    MotorError moveToNoWait(double angle_degrees);

    /**
     * @brief [0x23] Relative move by angle.
     */
    MotorError moveRelative(double delta_degrees);

    /**
     * @brief [0x21] Run at target velocity.
     */
    MotorError setVelocity(double velocity_rpm);

    /**
     * @brief [0x0F] Clear faults.
     */
    MotorError clearFaults();

    /**
     * @brief [0x1D] Set current position as zero.
     */
    MotorError setZero();

    /**
     * @brief [0x2E] Brake control.
     * @param closed True to close (apply brake), False to open (release brake).
     */
    MotorError setBrake(bool closed);

    /**
     * @brief Sets the smoothing factor for position control (EMA filter).
     * @param alpha Smoothing factor between 0.0 and 1.0.
     *              1.0 = No smoothing (instant follow)
     *              0.1 = Heavy smoothing
     *              0.0 = Target never changes
     */
    void setSmoothingFactor(double alpha);

    /**
     * @brief Returns the current smoothing factor.
     */
    double getSmoothingFactor() const { return smoothing_factor_; }

    /**
     * @brief Sets the speed limit for position control mode.
     * Uses command 0x15 to update motion control parameters.
     */
    MotorError setPositionSpeedLimit(double rpm);

    /**
     * @brief Sets PID parameters for position and velocity loops.
     */
    MotorError setPIDs(float pos_kp, float pos_ki, float vel_kp, float vel_ki);

    /**
     * @brief Gets current PID parameters.
     */
    MotorError getPIDs(float& pos_kp, float& pos_ki, float& vel_kp, float& vel_ki);

    /**
     * @brief A blocking version of moveTo.
     * Commands the motor to move and waits until the target is reached, a fault occurs,
     * or a timeout expires.
     * @param angle_degrees The target absolute angle in degrees.
     * @param timeout_ms The maximum time to wait in milliseconds.
     * @param tolerance_deg The acceptable position tolerance in degrees to consider the move complete.
     * @return MotorError::Ok if the target is reached successfully.
     * @return MotorError::Timeout if the motor did not reach the target in time.
     * @return MotorError::DeviceReportedFault if a fault occurred during the move.
     */
    MotorError moveToAndWait(double angle_degrees, unsigned int timeout_ms, double tolerance_deg = 0.1);

    /**
     * @brief High-level position control with a trapezoidal velocity profile.
     * This is a blocking call that implements a host-side trajectory generator,
     * sending velocity commands to the motor. It supports on-the-fly retargeting
     * if called again from another thread while a move is in progress.
     * @param angle_degrees Target absolute position in degrees.
     * @param profile Control parameters (max velocity, acceleration, etc.).
     * @param timeout_ms Maximum time to wait for reaching the target.
     * @return MotorError::Ok on success, or error code.
     */
    MotorError moveToWithProfile(double angle_degrees, const VelocityControlProfile& profile, unsigned int timeout_ms);

    // --- MIT Motion Control ---

    /**
     * @brief Configures limits for MIT Motion Control mode.
     * @param p_max Max Position (rad). Default 95.5.
     * @param v_max Max Velocity (rad/s). Default 45.0.
     * @param t_max Max Torque (Nm). Default 18.0.
     */
    MotorError setMitLimits(float p_max, float v_max, float t_max);

    /**
     * @brief Sends a real-time MIT Motion Control command.
     * Motor must be capable of receiving this command (no special mode switch needed for first command,
     * but limits should be set beforehand if defaults are not suitable).
     * 
     * @param p_des Target Position (rad)
     * @param v_des Target Velocity (rad/s)
     * @param kp Position Gain (0-500)
     * @param kd Velocity Gain (0-5)
     * @param t_ff Feedforward Torque (Nm)
     * @param[out] telemetry Updated telemetry from the response.
     * @return MotorError::Ok on success.
     */
    MotorError sendMitControl(float p_des, float v_des, float kp, float kd, float t_ff, Telemetry& telemetry);

    // --- Data Acquisition ---

    /**
     * @brief Fetches the latest telemetry data from the motor and converts it to physical units.
     * @param[out] telemetry A Telemetry struct to be filled with the data.
     * @return MotorError::Ok on success.
     */
    MotorError getTelemetry(Telemetry& telemetry);

    /**
     * @brief Optimized telemetry fetch for control loops. Only reads position data.
     * @param[out] angle_deg Filled with multi-turn angle in degrees.
     * @return MotorError::Ok on success.
     */
    MotorError getPositionFeedback(double& angle_deg);

    /**
     * @brief Checks if the motor has an active fault condition.
     * This reads the fault status from the device.
     * @return True if a fault is active, false otherwise.
     */
    bool hasFault();

    /**
     * @brief Converts a raw RealtimeDataPayload into a user-friendly Telemetry struct.
     * Public static method for convenience, can be used in tests or advanced scenarios.
     * @param payload The raw data payload from the protocol layer.
     * @return A Telemetry struct with converted values.
     */
    static Telemetry convertPayloadToTelemetry(const RealtimeDataPayload& payload);

    // --- Active Control Loop Methods ---

    /**
     * @brief Main control loop update. Must be called cyclically (e.g. at 100Hz+).
     * Handles state interpolation, sine wave generation, protections, and CAN transmission.
     * @param dt_seconds Time elapsed since last call.
     */
    void update(double dt_seconds);

    enum class ControlMode {
        RequestDisable, // Transition state to ensure disable command is sent
        Disabled,
        Hold,           // Active holding (servo on)
        Position,       // Standard Position Control (Interpolated)
        Velocity,       // Velocity Control
        Sinusoid,       // Internal Sine Wave Generator
        Mit             // MIT Impedance Control
    };

    /**
     * @brief Set the desired control mode.
     */
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return control_mode_; }

    /**
     * @brief Set target for Position or Velocity modes.
     */
    void setTargetAngle(double angle_deg) { target_angle_deg_ = angle_deg; }
    void setTargetVelocity(double vel_rpm) { target_velocity_rpm_ = vel_rpm; }

    struct SineParams {
        double amplitude_deg = 10.0;
        double frequency_hz = 0.5;
        double center_deg = 0.0;
        double phase_offset_rad = 0.0;
    };
    void setSineParams(const SineParams& params) { sine_params_ = params; }
    SineParams getSineParams() const { return sine_params_; }

    void setProtectionLimits(double min_voltage, double max_temp_c) {
        min_voltage_limit_ = min_voltage;
        max_temp_limit_ = max_temp_c;
    }

private:
    uint8_t device_address_;
    std::shared_ptr<SteadywinProtocol> protocol_;
    std::recursive_mutex& bus_mutex_;

    // --- State variables ---
    bool is_initialized_{false};
    Telemetry last_telemetry_{};
    SteadywinProtocolCAN::MitParams mit_limits_{}; // Default constructed

    // --- Control State ---
    ControlMode control_mode_{ControlMode::Disabled};
    ControlMode requested_mode_{ControlMode::Disabled};
    
    // Limits & Protections
    double min_voltage_limit_{10.0}; // Default safe limit
    double max_temp_limit_{80.0};
    double pos_speed_limit_rpm_{3000.0};

    // Interpolation / Command Generation
    double target_angle_deg_{0.0};       // User desired target
    double current_cmd_angle_{0.0};      // Actual command sent (ramped/interpolated)
    double target_velocity_rpm_{0.0};
    
    SineParams sine_params_;
    double sine_time_accumulator_{0.0};

    // --- Smoothing (EMA filter) variables ---
    double smoothing_factor_{1.0};
    double accumulated_target_deg_{0.0};
    bool target_initialized_{false};

    // --- Constants for unit conversion ---
    static constexpr double COUNTS_TO_DEG = 360.0 / 16384.0;
    static constexpr double DEG_TO_COUNTS = 16384.0 / 360.0;
    static constexpr double RPM_UNIT = 0.01;
    static constexpr double CURRENT_UNIT = 0.001;
    static constexpr double VOLTAGE_UNIT = 0.01;
};

} // namespace steadywin
