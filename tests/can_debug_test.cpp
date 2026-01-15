#include "steadywin/hal/gs_usb_can_port.h"
#include "steadywin/protocol/steadywin_protocol_can.h"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

using namespace steadywin;

void print_realtime_data(const RealtimeDataPayload& data) {
    std::cout << "\n--- Motor Telemetry ---" << std::endl;
    std::cout << "Position (counts): " << data.multi_turn_angle << std::endl;
    std::cout << "Velocity (RPM):    " << (float)data.mechanical_velocity / 100.0f << std::endl;
    std::cout << "Current (A):       " << (float)data.q_axis_current / 1000.0f << std::endl;
    std::cout << "Voltage (V):       " << (float)data.bus_voltage / 100.0f << std::endl;
    std::cout << "Temperature (C):   " << (int)data.working_temperature << std::endl;
    std::cout << "Fault Code:        0x" << std::hex << (int)data.fault_code << std::dec << std::endl;
}

int main() {
    std::cout << "Starting FINAL STEADYWIN DEBUG TEST (gs_usb)..." << std::endl;

    auto port = std::make_shared<GsUsbCanPort>();
    auto devices = port->enumerateDevices();

    if (devices.empty()) {
        std::cerr << "No gs_usb devices found!" << std::endl;
        return 1;
    }

    if (!port->open(devices[0].path.c_str(), 1000000)) {
        std::cerr << "Failed to open port!" << std::endl;
        return 1;
    }

    SteadywinProtocolCAN protocol(port);
    uint8_t motor_id = 0x01; 

    std::cout << "Reading telemetry from Motor ID " << (int)motor_id << " for 10 seconds..." << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
        RealtimeDataPayload data;
        if (protocol.readRealtimeData(motor_id, data) == MotorError::Ok) {
            print_realtime_data(data);
        } else {
            std::cout << "Timeout..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    port->close();
    std::cout << "\nTest finished." << std::endl;
    return 0;
}
