#pragma once

#include <cstdint>
#include <vector>

namespace steadywin {

/**
 * @struct CanFrame
 * @brief Represents a standard CAN 2.0B frame.
 */
struct CanFrame {
    uint32_t id{0};           // CAN Identifier (11-bit or 29-bit)
    uint8_t  dlc{0};          // Data Length Code (0-8)
    uint8_t  data[8]{0};      // Data payload
    bool     is_extended{false}; // True for 29-bit ID, False for 11-bit ID
    bool     is_rtr{false};      // Remote Transmission Request
};

/**
 * @class ICanPort
 * @brief An abstract interface for a CAN bus communication channel.
 */
class ICanPort {
public:
    virtual ~ICanPort() = default;

    /**
     * @brief Opens the CAN port.
     * @param channel The device name or channel index.
     * @param baud_rate The communication speed (e.g., 1000000 for 1Mbps).
     * @return True on success, false on failure.
     */
    virtual bool open(const char* channel, unsigned int baud_rate) = 0;

    /**
     * @brief Closes the CAN port.
     */
    virtual void close() = 0;

    /**
     * @brief Checks if the port is open.
     * @return True if the port is open, false otherwise.
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Sends a CAN frame.
     * @param frame The frame to send.
     * @return True on success, false on failure.
     */
    virtual bool write(const CanFrame& frame) = 0;

    /**
     * @brief Reads a CAN frame.
     * @param frame The frame to store the read data into.
     * @param timeout_ms Timeout for the read operation in milliseconds.
     * @return True if a frame was read, false on error or timeout.
     */
    virtual bool read(CanFrame& frame, unsigned int timeout_ms) = 0;
};

} // namespace steadywin
