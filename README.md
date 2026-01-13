# Steadywin Motor Control 🤖

![License](https://img.shields.io/badge/license-GPL--v3-blue.svg)
![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![Status](https://img.shields.io/badge/status-Stable-green.svg)

**Steadywin Motor Control** is a high-performance C++ library for interfacing with Steadywin motors via RS485 and CAN protocols. It provides a reliable abstraction layer for motor telemetry, control, and error handling.

![Interface](docs/program%20interface.png)

## 🏗 Architecture

The project is organized into the `steadywin_connect` directory, structured by layers:

### 1. Protocol Layer (`steadywin_connect/src/protocol`)
Implementations of communication protocols.
*   **`SteadywinProtocolRS485`**: Steadywin RS485 binary protocol.
*   **`SteadywinProtocolCAN`**: Custom CAN Communication Protocol V3.07b0.
*   **`SteadywinProtocol`**: Base abstract class for protocols.

### 2. Hardware Abstraction Layer (`steadywin_connect/src/hal`)
Interfaces and hardware-specific implementations.
*   **`ISerialPort` / `ICanPort`**: Communication interfaces.
*   **`WindowsSerialPort`**: Windows-specific serial implementation.
*   **`GSUsbCanPort` / `SerialCanPort`**: CAN adapter implementations.

### 3. Core Layer (`steadywin_connect/src/core`)
High-level motor control logic.
*   **`SteadywinMotor`**: Main class for motor interaction.
*   **`MotorError`**: Error definitions and handling.

### 4. Includes (`steadywin_connect/include/steadywin`)
All public header files are located here.

---

## 📖 Documentation

*   [**User Manual**](docs/usage_manual.md) - Detailed guide on how to use the software and hardware connection.
*   [CAN Protocol](docs/CAN_steadywin_protocol_v302.md) - Official CAN protocol specification.

---

## 🚀 Getting Started

### Prerequisites
*   Windows 10/11
*   CMake 3.10+
*   GCC or equivalent C++17 compiler
*   RS485 to USB adapter OR CAN to USB adapter

### Build
```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

### Usage (C++)
```cpp
// For RS485
auto port = std::make_shared<WindowsSerialPort>();
port->open("COM3", 115200);
auto protocol = std::make_unique<SteadywinProtocolRS485>(port);
SteadywinMotor motor(1, std::move(protocol));
motor.initialize();
motor.moveTo(90.0);

// For CAN
// auto port = std::make_shared<YourCanPortImplementation>();
// auto protocol = std::make_unique<SteadywinProtocolCAN>(port);
// SteadywinMotor motor(1, std::move(protocol));
```

---
