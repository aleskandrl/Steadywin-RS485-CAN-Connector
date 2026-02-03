// --- Файл: include/steadywin/serial_port_interface.h ---
#pragma once

#include <cstddef>
#include <vector>
#include <string_view>
#include <cstdint>

namespace steadywin {

/**
 * @class ISerialPort
 * @brief An abstract interface for a serial port communication channel.
 * This allows the motor controller to be platform-independent. A concrete
 * implementation for a specific OS (e.g., LinuxSerialPort, WindowsSerialPort)
 * must be provided by the user.
 */
class ISerialPort {
public:
    virtual ~ISerialPort() = default;

    /**
     * @brief Opens the serial port.
     * @param port_name The device name (e.g., "/dev/ttyUSB0" on Linux, "COM3" on Windows).
     * @param baud_rate The communication speed (e.g., 115200).
     * @return True on success, false on failure.
     */
    virtual bool open(std::string_view port_name, unsigned int baud_rate) = 0;

    /**
     * @brief Closes the serial port.
     */
    virtual void close() = 0;

    /**
     * @brief Checks if the port is open.
     * @return True if the port is open, false otherwise.
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Writes a block of bytes to the serial port.
     * @param data The buffer of bytes to write.
     * @return The number of bytes successfully written, or -1 on error.
     */
    virtual long long write(const std::vector<uint8_t>& data) = 0;

    /**
     * @brief Reads a block of bytes from the serial port.
     * @param buffer The buffer to store the read data into.
     * @param timeout_ms Timeout for the read operation in milliseconds.
     * @param max_bytes The maximum number of bytes to read. If 0, reads whatever is available up to internal buffer size.
     * @return The number of bytes read, or -1 on error/timeout.
     */
    virtual long long read(std::vector<uint8_t>& buffer, unsigned int timeout_ms, size_t max_bytes = 0) = 0;

    /**
     * @brief Flushes the input and output buffers of the serial port.
     */
    virtual void flush() = 0;
};

} // namespace steadywin
