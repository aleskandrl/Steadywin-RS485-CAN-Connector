#pragma once

#include <cstdint>

namespace steadywin {

// --- Protocol-level Data Payloads ---
// These structs map directly to the byte layout in the RS485 protocol.
// The #pragma pack directive ensures that the compiler does not add any padding
// bytes, which is crucial for correct parsing via memory mapping (e.g., memcpy).

#pragma pack(push, 1)

/**
 * @struct MotionControlParametersPayload
 * @brief Represents the data payload of the 0x14/0x15/0x16 commands.
 */
struct MotionControlParametersPayload {
    float    pos_kp;
    float    pos_ki;
    uint32_t pos_limit_rpm_x100; // Position mode max speed, Unit: 0.01 Rpm
    float    vel_kp;
    float    vel_ki;
    uint32_t vel_limit_ma;      // Max Q-axis current, Unit: 0.001A
};

/**
 * @struct RealtimeDataPayload
 * @brief Represents the data payload of the 0x0B (Read Real-time Data) response.
 * All values are raw, as received from the device.
 */
struct RealtimeDataPayload {
    uint16_t single_turn_angle;     // Raw counts (0-16383)
    int32_t  multi_turn_angle;      // Raw counts
    int32_t  mechanical_velocity;   // Unit: 0.01 Rpm
    int32_t  q_axis_current;        // Unit: 0.001A
    uint16_t bus_voltage;           // Unit: 0.01V
    uint16_t bus_current;           // Unit: 0.01A
    uint8_t  working_temperature;   // Unit: Celsius
    uint8_t  running_status;        // 0: Off, 2: Current, 3: Velocity, 4: Position
    uint8_t  motor_status;          // 0: Disabled, Other: Enabled
    uint8_t  fault_code;            // Bitmask of faults
};

#pragma pack(pop)


// --- High-level API Data Structures ---
// These structs are used in the main SteadywinMotor class for user interaction.
// They contain values in standard physical units (degrees, RPM, Amps, etc.).

/**
 * @struct VelocityControlProfile
 * @brief Encapsulates parameters for the high-level position controller.
 */
struct VelocityControlProfile {
    double p_gain{1.0};            // Proportional gain
    double i_gain{0.0};            // Integral gain
    double d_gain{0.0};            // Derivative gain
    double max_velocity_rpm{100.0}; // Maximum velocity limit in RPM
    double acceleration_rpm_s{500.0}; // Acceleration limit in RPM/s
    double tolerance_deg{0.1};      // Target position tolerance in degrees
};

/**
 * @struct Telemetry
 * @brief Holds all real-time telemetry data in user-friendly physical units.
 */
struct Telemetry {
    double   single_turn_angle_deg{0.0};    // Angle within one rotation, in degrees.
    double   multi_turn_angle_deg{0.0};     // Absolute cumulative angle, in degrees.
    double   velocity_rpm{0.0};             // Velocity in Revolutions Per Minute.
    double   q_axis_current_amps{0.0};      // Q-axis current (proportional to torque) in Amperes.
    double   bus_voltage_volts{0.0};        // Bus voltage in Volts.
    double   bus_current_amps{0.0};         // Bus current in Amperes.
    int8_t   temperature_celsius{0};        // Working temperature in Celsius.
    uint8_t  raw_running_status{0};         // Raw running status code.
    uint8_t  raw_motor_status{0};           // Raw motor enabled/disabled status.
    uint8_t  raw_fault_code{0};             // Raw fault bitmask.
};

} // namespace steadywin
