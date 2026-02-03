#pragma once

#include "steadywin/protocol/motor_error.h"
#include "steadywin/protocol/steadywin_protocol.h"
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

private:
    uint8_t device_address_;
    std::shared_ptr<SteadywinProtocol> protocol_;
    std::recursive_mutex& bus_mutex_;

    // --- State variables ---
    bool is_initialized_{false};
    Telemetry last_telemetry_{};

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
