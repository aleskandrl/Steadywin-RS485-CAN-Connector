#include "steadywin/windows_serial_port.h"
#include <iostream>
#include <iomanip>
#include <vector>

namespace steadywin {

WindowsSerialPort::~WindowsSerialPort() {
    close();
}

bool WindowsSerialPort::open(std::string_view port_name, unsigned int baud_rate) {
    std::string name = "\\\\.\\";
    name += port_name;

    hSerial_ = CreateFileA(name.c_str(),
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);

    if (hSerial_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial_, &dcbSerialParams)) {
        CloseHandle(hSerial_);
        return false;
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial_, &dcbSerialParams)) {
        CloseHandle(hSerial_);
        return false;
    }

    is_open_ = true;
    return true;
}

void WindowsSerialPort::close() {
    if (hSerial_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial_);
        hSerial_ = INVALID_HANDLE_VALUE;
    }
    is_open_ = false;
}

bool WindowsSerialPort::isOpen() const {
    return is_open_;
}

long long WindowsSerialPort::write(const std::vector<uint8_t>& data) {
    if (!is_open_) return -1;

    // printHex("TX -> ", data.data(), data.size());

    DWORD bytesWritten;
    if (!WriteFile(hSerial_, data.data(), data.size(), &bytesWritten, NULL)) {
        return -1;
    }
    return static_cast<long long>(bytesWritten);
}

long long WindowsSerialPort::read(std::vector<uint8_t>& buffer, unsigned int timeout_ms) {
    if (!is_open_) return -1;

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial_, &timeouts);

    uint8_t temp_buf[1024];
    DWORD bytesRead;
    if (!ReadFile(hSerial_, temp_buf, sizeof(temp_buf), &bytesRead, NULL)) {
        return -1;
    }

    if (bytesRead > 0) {
        buffer.assign(temp_buf, temp_buf + bytesRead);
        // printHex("RX <- ", buffer.data(), buffer.size());
    }

    return static_cast<long long>(bytesRead);
}

void WindowsSerialPort::printHex(std::string_view prefix, const uint8_t* data, size_t length) {
    std::cout << prefix;
    for (size_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

} // namespace steadywin
