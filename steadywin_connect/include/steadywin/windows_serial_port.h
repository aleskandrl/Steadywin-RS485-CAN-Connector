#pragma once

#include "steadywin/serial_port_interface.h"
#include <windows.h>
#include <string>

namespace steadywin {

/**
 * @class WindowsSerialPort
 * @brief Implementation of ISerialPort for Windows using Win32 API.
 */
class WindowsSerialPort : public ISerialPort {
public:
    WindowsSerialPort() = default;
    ~WindowsSerialPort() override;

    bool open(std::string_view port_name, unsigned int baud_rate) override;
    void close() override;
    bool isOpen() const override;
    long long write(const std::vector<uint8_t>& data) override;
    long long read(std::vector<uint8_t>& buffer, unsigned int timeout_ms) override;

private:
    HANDLE hSerial_{INVALID_HANDLE_VALUE};
    bool is_open_{false};
    
    void printHex(std::string_view prefix, const uint8_t* data, size_t length);
};

} // namespace steadywin
