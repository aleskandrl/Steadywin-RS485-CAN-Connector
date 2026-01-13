# Steadywin Motor Control User Manual 🦾

This document describes the process of configuring and using the software for controlling Steadywin motors via RS485 and CAN interfaces.

## 📥 Getting Started

1.  **Hardware Connection:**
    *   Connect the motor to the power supply.
    *   Connect the adapter (RS485-USB or CAN-USB) to your computer.
    *   Connect the adapter to the motor according to the pinout diagram.

2.  **Drivers:**
    *   Ensure that the drivers for your adapter are installed in the system (e.g., CP210x for RS485 or WinUSB/Zadig for CAN adapters like CandleLight).

## 🚀 Running the Program

The program can be run via the compiled executable or via a Python script (for testing).

### Using the GUI (steadywin_connector)

1.  Run `steadywin_connector.exe`.
2.  Select the interface type (**RS485** or **CAN**).
3.  Specify the connection parameters:
    *   For RS485: COM port and baud rate (usually 115200).
    *   For CAN: Channel number or device ID.
4.  Click **Connect**. Upon successful connection, the indicator will turn green.

## ⚙️ Core Functions

*   **Initialize:** Puts the motor into an operational state.
*   **Move To:** Set target position (in degrees).
*   **Set Velocity:** Set rotation speed (RPM).
*   **Brake Toggle:** Toggle the electromagnetic brake state.

## ⚠️ Important Note on Brake Control

> [!CAUTION]
> **WARNING:** The software **does not have feedback** about the current physical state of the brake upon power-up or initialization.
> 
> *   The system **does not know** whether the brake is currently engaged or disengaged.
> *   The program simply sends a command to **change the state** or set a specific value.
> *   Be extremely careful during the first power-up: ensure the current brake state matches your expectations to avoid sudden movements or mechanical damage.

## 🛠 Troubleshooting

*   **Motor not responding:** Check the correctness of the A/B wire connection (for RS485) or CAN_H/CAN_L. Try swapping them.
*   **Timeout Error:** Ensure the correct motor ID is selected (default is usually 1).
*   **Console Errors:** Check if the port is occupied by another application.
