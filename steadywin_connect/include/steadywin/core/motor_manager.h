#pragma once

#include "steadywin/core/steadywin_motor.h"
#include "steadywin/protocol/steadywin_protocol.h"
#include "steadywin/hal/can_port_interface.h"
#include "steadywin/hal/serial_port_interface.h"
#include <map>
#include <memory>
#include <vector>
#include <mutex>
#include <cstdint>

namespace steadywin {

/**
 * @class MotorManager
 * @brief Manages the communication bus and all connected Steadywin motors.
 * This class is the primary entry point for controlling a group of motors.
 * It handles bus scanning, device discovery, and provides synchronized access
 * to the communication port, ensuring thread safety.
 */
class MotorManager {
public:
    /**
     * @brief Constructor for CAN-based communication.
     * @param can_port A shared pointer to an initialized ICanPort implementation.
     */
    explicit MotorManager(std::shared_ptr<ICanPort> can_port);

    /**
     * @brief Constructor for RS485-based communication.
     * @param serial_port A shared pointer to an initialized ISerialPort implementation.
     */
    explicit MotorManager(std::shared_ptr<ISerialPort> serial_port);

    /**
     * @brief Scans the bus for available motors and initializes them.
     * This method must be called before attempting to get a motor.
     * It pings each address from 1 up to max_address_to_scan.
     * @param max_address_to_scan The highest device address to check (e.g., 10).
     * @return A vector of device addresses that were found and initialized.
     */
    std::vector<uint8_t> scanBus(uint8_t max_address_to_scan = 10);

    /**
     * @brief Retrieves a pointer to a specific motor controller.
     * @param device_address The address of the motor to get.
     * @return A shared pointer to the SteadywinMotor object, or nullptr if not found.
     */
    std::shared_ptr<SteadywinMotor> getMotor(uint8_t device_address);

    /**
     * @brief Gets a list of all device addresses found during the last scan.
     * @return A vector of active device addresses.
     */
    std::vector<uint8_t> getFoundMotorIds() const;
    
    /**
     * @brief Disables all motors found on the bus.
     * A convenience function for shutting down the entire system.
     */
    void disableAllMotors();

private:
    std::shared_ptr<SteadywinProtocol> protocol_;
    std::recursive_mutex bus_mutex_;
    std::map<uint8_t, std::shared_ptr<SteadywinMotor>> motors_;
    bool is_initialized_{false};
};

} // namespace steadywin