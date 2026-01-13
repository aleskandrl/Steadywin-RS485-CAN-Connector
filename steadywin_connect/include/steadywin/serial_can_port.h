#pragma once

#include "steadywin/can_port_interface.h"
#include "steadywin/serial_port_interface.h"
#include <memory>
#include <string>

namespace steadywin {

/**
 * @class SerialCanPort
 * @brief Implementation of ICanPort for serial-to-CAN adapters using SLCAN protocol.
 */
class SerialCanPort : public ICanPort {
public:
    explicit SerialCanPort(std::shared_ptr<ISerialPort> serial_port);
    ~SerialCanPort() override;

    bool open(const char* channel, unsigned int baud_rate) override;
    void close() override;
    bool isOpen() const override;
    bool write(const CanFrame& frame) override;
    bool read(CanFrame& frame, unsigned int timeout_ms) override;

private:
    std::shared_ptr<ISerialPort> serial_port_;
    bool is_open_{false};
};

} // namespace steadywin
