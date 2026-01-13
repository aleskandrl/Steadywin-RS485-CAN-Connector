#include "steadywin/serial_can_port.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <sstream>
#include <iomanip>

namespace steadywin {

SerialCanPort::SerialCanPort(std::shared_ptr<ISerialPort> serial_port)
    : serial_port_(std::move(serial_port)) {}

SerialCanPort::~SerialCanPort() {
    close();
}

bool SerialCanPort::open(const char* channel, unsigned int baud_rate) {
    if (!serial_port_->open(channel, baud_rate)) return false;
    
    // SLCAN initialization (open command 'O')
    std::vector<uint8_t> open_cmd = {'O', '\r'};
    serial_port_->write(open_cmd);
    
    is_open_ = true;
    return true;
}

void SerialCanPort::close() {
    if (is_open_) {
        std::vector<uint8_t> close_cmd = {'C', '\r'};
        serial_port_->write(close_cmd);
        serial_port_->close();
    }
    is_open_ = false;
}

bool SerialCanPort::isOpen() const {
    return is_open_ && serial_port_->isOpen();
}

bool SerialCanPort::write(const CanFrame& frame) {
    if (!isOpen()) return false;

    std::stringstream ss;
    if (frame.is_extended) {
        ss << (frame.is_rtr ? 'R' : 'T');
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << frame.id;
    } else {
        ss << (frame.is_rtr ? 'r' : 't');
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.id;
    }
    
    ss << std::dec << static_cast<int>(frame.dlc);
    
    for (int i = 0; i < frame.dlc; ++i) {
        ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(frame.data[i]);
    }
    ss << '\r';

    std::string cmd = ss.str();
    std::vector<uint8_t> data(cmd.begin(), cmd.end());
    return serial_port_->write(data) == static_cast<long long>(data.size());
}

bool SerialCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    if (!isOpen()) return false;

    std::vector<uint8_t> buffer;
    if (serial_port_->read(buffer, timeout_ms) <= 0) return false;

    // Very basic SLCAN parser (expects a full command in one read for simplicity)
    if (buffer.empty()) return false;
    
    std::string s(buffer.begin(), buffer.end());
    if (s.length() < 5) return false;

    char type = s[0];
    if (type == 't' || type == 'r') {
        frame.is_extended = false;
        frame.is_rtr = (type == 'r');
        frame.id = std::stoul(s.substr(1, 3), nullptr, 16);
        frame.dlc = std::stoi(s.substr(4, 1), nullptr, 16);
        for (int i = 0; i < frame.dlc; ++i) {
            frame.data[i] = static_cast<uint8_t>(std::stoul(s.substr(5 + i * 2, 2), nullptr, 16));
        }
        return true;
    } else if (type == 'T' || type == 'R') {
        frame.is_extended = true;
        frame.is_rtr = (type == 'R');
        frame.id = std::stoul(s.substr(1, 8), nullptr, 16);
        frame.dlc = std::stoi(s.substr(9, 1), nullptr, 16);
        for (int i = 0; i < frame.dlc; ++i) {
            frame.data[i] = static_cast<uint8_t>(std::stoul(s.substr(10 + i * 2, 2), nullptr, 16));
        }
        return true;
    }

    return false;
}

} // namespace steadywin
