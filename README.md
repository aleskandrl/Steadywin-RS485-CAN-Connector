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

### Run Apps

- ⚠️ **Current status:** `telemetry_6motors_app` is the most up-to-date application in this repository.
  
  **Use at your own risk** on real hardware. Always keep external E-stop and mechanical safety measures active.

- Main example app:
```bash
build\example_app.exe
```

- Telemetry + 6 motors control UI:
```bash
build\telemetry_6motors_app.exe
```

### Usage (C++)
```cpp
// Recommended flow: use MotorManager, then get motor instances.
auto port = std::make_shared<WindowsSerialPort>();
port->open("COM3", 115200);

steadywin::MotorManager manager(port);
auto ids = manager.scanBus(10);
if (!ids.empty()) {
    auto motor = manager.getMotor(ids.front());
    if (motor) {
        motor->moveTo(90.0);
    }
}
```

---

## 🛡 Safety Behavior (telemetry_6motors_app)

`telemetry_6motors_app` includes startup and runtime safety gates to reduce unintended motion risk:

1. **Pre-arm disable sweep (CAN/RS485)**
   - Right after opening the port, app sends `disable` over address range (default `1..20`) before mode selection.

2. **Mandatory ARM step**
   - After scan and initial disable handshake, user must explicitly choose:
     - `ARM and continue`
     - or `Abort (safe exit)`

3. **Hard runtime command gate**
   - Commands are allowed only when:
     - app is armed, and
     - axis is explicitly enabled.
   - If axis is not allowed, it is forced to `Disabled/RequestDisable` and sine command stream is dropped.

4. **Sine safety rules**
   - `M` (toggle sine) works only for enabled axis.
   - `F` (disable active axis) immediately clears sine on that axis.

5. **UI safety indicators**
   - `Armed:Y/N`
   - `CmdGate:Y/N` for active axis

### Recommended Safe Startup Sequence

1. Ensure external E-stop is available.
2. Start `telemetry_6motors_app`.
3. Select transport (CAN/RS485) and wait for pre-arm disable sweep.
4. Select mode.
5. Confirm `ARM and continue`.
6. Enable only required axis with `E`.
7. Start motion commands (`A/D` or `M` for sine) only after enable.



