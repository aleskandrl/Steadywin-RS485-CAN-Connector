// --- Файл: include/steadywin/motor_error.h ---
#pragma once

#include <cstdint>

namespace steadywin {

/**
 * @enum MotorError
 * @brief Defines all possible error codes for the motor control library.
 * This enum is used as a return type for functions that can fail.
 */
enum class MotorError : uint8_t {
    // --- Generic Errors (0-19) ---
    Ok = 0,                 // Success.
    UnknownError,           // An unspecified error occurred.

    // --- Communication Errors (20-39) ---
    PortNotOpen = 20,       // The serial port is not open.
    WriteError,             // Failed to write data to the port.
    ReadError,              // Failed to read data from the port.
    Timeout,                // The operation timed out waiting for a response.
    InvalidResponse,        // The received response packet is malformed.
    CrcMismatch,            // The CRC checksum of the received packet is incorrect.
    PacketSequenceMismatch, // The sequence number in the response does not match the request.

    // --- Device-Reported Errors (40-59) ---
    // These correspond to the fault codes from the device (Commands 0x0B, 0x0F)
    DeviceReportedFault = 40, // A generic fault reported by the device.
    DeviceVoltageFault,     // Voltage fault (Bit0).
    DeviceCurrentFault,     // Current fault (Bit1).
    DeviceTemperatureFault, // Temperature fault (Bit2).
    DeviceEncoderFault,     // Encoder fault (Bit3).
    DeviceHardwareFault,    // Hardware fault (Bit6).
    DeviceSoftwareFault,    // Software fault (Bit7).

    // --- API Usage Errors (60-79) ---
    InvalidArguments = 60,  // Invalid arguments provided to a method.
    NotInitialized,         // The motor object has not been initialized.
    FeatureNotEnabled,      // Attempted to use a feature that is disabled (e.g., second encoder).
    OperationFailed,        // The command was sent, but the device failed to execute it.
};

} // namespace steadywin