#include "steadywin/hal/gs_usb_can_port.h"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>
#include <cstring>

using namespace steadywin;

void print_frame(const std::string& prefix, const CanFrame& frame) {
    std::cout << prefix 
              << " ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.id
              << " DLC: " << std::dec << (int)frame.dlc
              << " Data: ";
    std::cout << std::hex << std::setfill('0');
    for (int i = 0; i < frame.dlc; ++i) {
        std::cout << std::setw(2) << (int)frame.data[i] << " ";
    }
    std::cout << std::dec << std::endl;
}

bool run_test(std::shared_ptr<GsUsbCanPort> port, uint8_t motor_id, bool use_new_protocol) {
    uint32_t tx_id = use_new_protocol ? (0x100 | motor_id) : motor_id;
    uint32_t rx_expect_id = motor_id;
    
    std::cout << "\n---------------------------------------------------" << std::endl;
    std::cout << "TESTING " << (use_new_protocol ? "NEW PROTOCOL (0x100 | ID)" : "OLD PROTOCOL (Direct ID)") << std::endl;
    std::cout << "Sending to ID: 0x" << std::hex << tx_id << std::dec << "..." << std::endl;

    // 1. Очистка буфера
    CanFrame junk;
    while(port->read(junk, 0)) {} 

    // 2. Подготовка команды (0xA0 - Read Version)
    CanFrame tx;
    tx.id = tx_id;
    tx.dlc = 1;
    tx.data[0] = 0xA0;

    if (!port->write(tx)) {
        std::cerr << "Write failed! (Check connection/power)" << std::endl;
        return false;
    }

    // 3. Чтение ответов
    bool success = false;
    auto start = std::chrono::steady_clock::now();
    
    // Слушаем 100мс
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
        CanFrame rx;
        if (port->read(rx, 10)) {
            
            // Анализ пакета
            if (rx.id == tx_id && rx.dlc == tx.dlc && rx.data[0] == tx.data[0]) {
                print_frame("[ECHO]  ", rx);
            } 
            else if (rx.id == rx_expect_id) {
                print_frame("[REPLY] ", rx);
                success = true;
                // Не выходим сразу, чтобы посмотреть, не придет ли что-то еще
            } 
            else {
                print_frame("[????]  ", rx);
            }
        }
    }

    if (success) std::cout << ">>> RESULT: SUCCESS! This protocol works. <<<" << std::endl;
    else std::cout << ">>> RESULT: No valid reply received. <<<" << std::endl;
    
    return success;
}

int main() {
    std::cout << "=== CAN PROTOCOL DIAGNOSTIC TOOL ===" << std::endl;

    auto port = std::make_shared<GsUsbCanPort>();
    auto devices = port->enumerateDevices();

    if (devices.empty()) {
        std::cerr << "No gs_usb devices found!" << std::endl;
        return 1;
    }

    // --- НАСТРОЙКИ ---
    uint8_t target_id = 1;      // ID мотора (обычно 1)
    unsigned int baud = 1000000; // Скорость (1M)
    // -----------------

    std::cout << "Opening device: " << devices[0].path << std::endl;
    if (!port->open(devices[0].path.c_str(), baud)) {
        std::cerr << "Failed to open port at " << baud << " baud!" << std::endl;
        return 1;
    }

    // Тест 1: Старый протокол
    run_test(port, target_id, false);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Тест 2: Новый протокол
    run_test(port, target_id, true);

    port->close();
    std::cout << "\nDiagnostic finished." << std::endl;
    return 0;
}