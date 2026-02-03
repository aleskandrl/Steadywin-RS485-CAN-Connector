#include "steadywin/core/motor_manager.h"
#include "steadywin/protocol/steadywin_protocol_can.h"
#include "steadywin/protocol/steadywin_protocol_rs485.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

namespace steadywin {

MotorManager::MotorManager(std::shared_ptr<ICanPort> can_port) {
    if (can_port && can_port->isOpen()) {
        protocol_ = std::make_shared<SteadywinProtocolCAN>(can_port);
        is_initialized_ = true;
    } else {
        // Handle error: Port is not valid or not open
        is_initialized_ = false;
    }
}

MotorManager::MotorManager(std::shared_ptr<ISerialPort> serial_port) {
    if (serial_port && serial_port->isOpen()) {
        protocol_ = std::make_shared<SteadywinProtocolRS485>(serial_port);
        is_initialized_ = true;
    } else {
        // Handle error: Port is not valid or not open
        is_initialized_ = false;
    }
}

std::vector<uint8_t> MotorManager::scanBus(uint8_t max_address_to_scan) {
    std::vector<uint8_t> found_devices;
    if (!is_initialized_) {
        std::cerr << "MotorManager is not initialized." << std::endl;
        return found_devices;
    }

    std::cout << "Scanning for motors up to address " << (int)max_address_to_scan << "..." << std::endl;
    
    // Clear any previously found motors
    motors_.clear();

    for (uint8_t i = 1; i <= max_address_to_scan; ++i) {
        RealtimeDataPayload temp_payload;
        MotorError err = MotorError::Timeout;
        
        // Retry logic: Try 3 times to find a motor
        for (int attempt = 0; attempt < 3; ++attempt) {
            {
                // Use a lock to ensure this scan operation is thread-safe,
                // even though it's typically called only once at the start.
                std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
                err = protocol_->readRealtimeData(i, temp_payload);
            }

            if (err == MotorError::Ok) break;
            
            // Short delay before retry to let the bus/buffers settle
            if (attempt < 2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (err == MotorError::Ok) {
            std::cout << "  Found motor at address " << (int)i << "." << std::endl;
            
            // Create and store the motor object
            auto motor = std::make_shared<SteadywinMotor>(i, protocol_, bus_mutex_);
            
            // Perform basic initialization to confirm it's fully responsive
            if (motor->initialize() == MotorError::Ok) {
                motors_[i] = motor;
                found_devices.push_back(i);
            } else {
                std::cerr << "  Motor at address " << (int)i << " found, but failed to initialize." << std::endl;
            }
        }
    }
    
    if (found_devices.empty()) {
        std::cout << "No motors found on the bus." << std::endl;
    }

    return found_devices;
}

std::shared_ptr<SteadywinMotor> MotorManager::getMotor(uint8_t device_address) {
    auto it = motors_.find(device_address);
    if (it != motors_.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<uint8_t> MotorManager::getFoundMotorIds() const {
    std::vector<uint8_t> ids;
    ids.reserve(motors_.size());
    for (const auto& pair : motors_) {
        ids.push_back(pair.first);
    }
    return ids;
}

void MotorManager::disableAllMotors() {
    for (auto const& [address, motor] : motors_) {
        if (motor) {
            motor->disable();
        }
    }
}

} // namespace steadywin