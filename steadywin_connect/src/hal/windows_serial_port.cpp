#include "steadywin/hal/windows_serial_port.h"
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

long long WindowsSerialPort::read(std::vector<uint8_t>& buffer, unsigned int timeout_ms, size_t max_bytes) {
    if (!is_open_) return -1;

    // Use a reasonable buffer size if max_bytes is not specified
    size_t bytes_to_request = (max_bytes > 0) ? max_bytes : 1024;

    COMMTIMEOUTS timeouts = {0};
    // ReadIntervalTimeout = 1ms ensures we don't wait 50ms after the last byte.
    // At 115200 baud, 1 byte takes ~0.087ms, so 1ms is plenty for inter-byte gap.
    timeouts.ReadIntervalTimeout = 1; 
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 0; // Don't add per-byte delay for small packets
    SetCommTimeouts(hSerial_, &timeouts);

    std::vector<uint8_t> temp_buf(bytes_to_request);
    DWORD bytesRead;
    if (!ReadFile(hSerial_, temp_buf.data(), static_cast<DWORD>(temp_buf.size()), &bytesRead, NULL)) {
        return -1;
    }

    if (bytesRead > 0) {
        buffer.assign(temp_buf.begin(), temp_buf.begin() + bytesRead);
        // printHex("RX <- ", buffer.data(), buffer.size());
    } else {
        buffer.clear();
    }

    return static_cast<long long>(bytesRead);
}

void WindowsSerialPort::flush() {
    if (is_open_ && hSerial_ != INVALID_HANDLE_VALUE) {
        PurgeComm(hSerial_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }
}

void WindowsSerialPort::printHex(std::string_view prefix, const uint8_t* data, size_t length) {
    std::cout << prefix;
    for (size_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

} // namespace steadywin
