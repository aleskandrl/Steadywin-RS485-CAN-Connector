
### Custom RS485 Communication Protocol_V3.02b4

**Protocol Version:**
*   **Rev.3.01b0 – 01.06.2024:** Initial version;
*   **Rev.3.01b1 – 25.06.2024:** Added more examples of communication data frames;
*   **Rev.3.02b0 – 21.11.2024:** Added support for Tamagawa RS485 protocol encoders; added holding brake output control command; added encoder faults to fault codes;
*   **Rev.3.02b1 – 20.02.2025:** Corrected errors in the command code field filling in the 0x10 command code table within the document;
*   **Rev.3.02b2 – 21.02.2025:** Corrected descriptions regarding enabling the second encoder in 0x10 and 0x11 commands;
*   **Rev.3.02b3 – 28.03.2025:** Improved communication examples;
*   **Rev.3.02b4 – 24.04.2025:** Optimized document content descriptions;

<br>



### Protocol Introduction:

The RS485 interface default communication baud rate is **115200** (Data bits: 8, Parity: 0, Stop bits: 1). The communication baud rate can be configured via the host computer's user parameter configuration interface to: 921600, 460800, 115200, 57600, 38400, 19200, 9600. All bytes use **Little-Endian** byte order.

The product's factory default **Device Address is 0x01**. The device address can be configured via the host computer to 1-254. Among them:
*   Device address **0** is the **Broadcast Address**: Slaves execute the command but do not reply to the Master.
*   Device address **255 (0xFF)** is the **Public Address**: All slaves respond to commands at this address. The Master can use this address to communicate with a single slave to retrieve configuration parameters.
*   The current device address can be obtained based on the flashing status of the green LED on the driver board (Device address 1: `[__ -__-__ ]`, Device address 2: `[ __-_-__-_-__]`, Device address 3: `[ __-_-_-__-_-_-__]`, etc.).

**Note:** The RS485 bus characteristics dictate that there can only be one Master on the bus, and Slave device addresses cannot be identical. If there are multiple Slaves with the same device address on the bus, and the Master communicates via that address, the Slaves with the same address will reply to the Master simultaneously, causing a bus short circuit and damaging the control board. Therefore, when multiple Slaves are mounted on the same bus, the device addresses of the Slaves must be configured in advance via the host computer or communication protocol.

**The command format for the RS485_V3.x version protocol communication data packet is as follows:**

| Field Name | Byte Count | Description |
| :--- | :--- | :--- |
| **Protocol Header** | 1 Byte | The protocol header sent by the Master is **0xAE**; the protocol header replied by the Slave is **0xAC**; |
| **Packet Sequence** | 1 Byte | The packet sequence number replied by the Slave is consistent with the packet sequence number sent by the Master to the Slave; |
| **Device Address** | 1 Byte | Address range is 0-0xFF; 0x00 is Broadcast Address; 0xFF is Public Address;<br>**Broadcast Address:** Slaves can receive and execute instructions but do not reply to the Master;<br>**Public Address:** Slaves can receive and execute instructions but do reply to the Master; Determined by RS485 bus characteristics, when multiple motors are connected to the bus, the Public Address cannot be used; |
| **Command Code** | 1 Byte | Different commands have different command codes; |
| **Data Packet Length** | 1 Byte | Number of bytes in the **[Data Field]** (can be 0); |
| **Data Field** | 0-248 (Bytes) | Data attached to the command code; |
| **CRC16 Check** | 2 Bytes | CRC16_MODBUS check value from **[Protocol Header]** to **[Data Field]**;<br>For checksum calculation, refer to the resource package "CRC16_MODBUS Check C Language Source Code" |

**Data Type Description:**
*   **1u** – 1 Unsigned Byte
*   **1s** – 1 Signed Byte
*   **2u** – 2 Unsigned Bytes (Little-Endian byte order)
*   **2s** – 2 Signed Bytes (Little-Endian byte order)
*   **4u** – 4 Unsigned Bytes (Little-Endian byte order)
*   **4s** – 4 Signed Bytes (Little-Endian byte order)
*   **4f** – Single-precision floating point number (Little-Endian byte order)
*   **Nb** – Byte array of size N

<br>


**RS485_3.x Version Protocol Control Commands are as follows:**

| Category | Command Code | Command Function Description |
| :--- | :--- | :--- |
| **System** | **0x00** | Reboot Slave. After the Master sends this command packet, the Slave reboots immediately and does not reply to the Master. |
| | **0x0A** | Read Boot, software, hardware, RS485 protocol, and CAN protocol versions and UID. |
| | **0x0B** | Read real-time information such as single-turn absolute angle, multi-turn absolute angle, velocity, Q-axis current, etc. |
| | **0x0F** | Clear faults. |
| **Parameters**| **0x10** | Read user parameters. |
| | **0x11** | Write and save user parameters. |
| | **0x12** | Read motor hardware parameters. |
| | **0x13** | Write and save motor hardware parameters. |
| | **0x14** | Read motion control parameters. |
| | **0x15** | Write motion control parameters but do not save. |
| | **0x16** | Write and save motion control parameters. |
| | **0x1D** | Set current position as origin (zero point). |
| | **0x1E** | Encoder calibration; The motor must be under no load during calibration, and motor rotation must not be interfered with. |
| | **0x1F** | Restore parameters to default values. |
| **Control** | **0x20** | Q-axis current control; (Torque = Torque Constant * Q-axis Current). |
| | **0x21** | Velocity control. |
| | **0x22** | Absolute position control. |
| | **0x23** | Relative position control. |
| | **0x24** | Motor returns to the set origin via the shortest distance; rotation angle is not greater than 180 degrees. |
| | **0x2E** | Holding brake control switch output control. |
| | **0x2F** | Turn off motor output. The motor enters a free state and is uncontrolled (this is the state after the motor is powered on). |

***


➢ **Reboot Slave:** After the Master sends this command packet, the Slave reboots immediately and does not reply to the Master; **【Command Code: 0x00】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x00** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

<br>

➢ **Read Boot, software, hardware, RS485 protocol, and CAN protocol versions, and Unique Serial Number;** **【Command Code: 0x0A】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x0A** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x0A** |
| [4] | Data Packet Length | 1u | 0x16 |
| [5]-[6] | Boot Software Version | 2u | |
| [7]-[8] | App Software Version | 2u | |
| [9]-[10] | Hardware Model | 2u | Driver board hardware code; |
| [11] | RS485-Custom | 1u | RS485-Custom protocol version; |
| [12] | RS485-Modbus | 1u | RS485-Modbus protocol version; (0 indicates not supported) |
| [13] | CAN-Custom | 1u | CAN-Custom protocol version; |
| [14] | CAN-CanOpen | 1u | CanOpen protocol version; (0 indicates not supported) |
| [15]-[26] | UID Unique Serial No. | 12b | Unique device serial number |
| [27]-[28] | CRC16 Check | 2u | DATA[0]~DATA[26] bytes CRC16 Check |

<br>

➢ **Read Real-time Data** (Single-turn absolute angle, Multi-turn absolute angle, velocity, Q-axis current, Bus voltage, Bus current, working temperature, running status, motor status, fault codes); Single-turn resolution is 14-bit; **【Command Code: 0x0B】**

*   **Master sends to Motor**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |


| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [3] | Command Code | 1u | **0x0B** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Motor (Slave) replies to Master Controller**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x0B** |
| [4] | Data Packet Length | 1u | 0x16 |
| [5]-[6] | Single-turn absolute angle | 2u | `Angle° = value*(360/16384)` |
| [7]-[10] | Multi-turn absolute angle | 4s | `Total Angle° = value*(360/16384)` |
| [11]-[14] | Mechanical Velocity | 4s | Unit 0.01 Rpm; |
| [15]-[18] | Q-axis Current | 4s | Unit 0.001A; (Torque = Q-axis current * Torque constant) |
| [19]-[20] | Bus Voltage | 2u | Unit 0.01V; |
| [21]-[22] | Bus Current | 2u | Unit 0.01A; Value is invalid during encoder calibration; |
| [23] | Working Temperature | 1u | Unit ℃; |
| [24] | Running Status | 1u | 0: Closed (Off) state<br>1: Voltage control<br>2: Q-axis current control<br>3: Velocity control<br>4: Position control |
| [25] | Motor Status | 1u | 0: Disabled<br>Other: Enabled |
| [26] | Fault Code | 1u | [Bit0]: Voltage fault<br>[Bit1]: Current fault<br>[Bit2]: Temperature fault<br>[Bit3]: Encoder fault<br>[Bit6]: Hardware fault<br>[Bit7]: Software fault<br>*(Bitn represents the nth bit of the byte; Bit0 represents the 0th bit)* |
| [27]-[28] | CRC16 Check | 2u | DATA[0]~DATA[26] bytes CRC16 Check |

<br>

➢ **Clear Faults;** **【Command Code: 0x0F】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x0F** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x0F** |
| [4] | Data Packet Length | 1u | 0x01 |


| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [5] | Current Fault | 1u | [Bit0]: Voltage fault; [Bit1]: Current fault<br>[Bit2]: Temperature fault; [Bit3]: Encoder fault<br>[Bit6]: Hardware fault; [Bit7]: Software fault<br>*(Bitn represents the nth bit of the byte; Bit0 represents the 0th bit)* |
| [6]-[7] | CRC16 Check | 2u | DATA[0]~DATA[5] bytes CRC16 Check |

<br>

➢ **Read User Parameters;** **【Command Code: 0x10】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x10** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x10** |
| [4] | Data Packet Length | 1u | 0x1A |
| [5]-[6] | Electrical Angle Offset | 2u | Angle offset obtained from encoder calibration; |
| [7]-[8] | Mechanical Angle Offset | 2u | Angle offset obtained from zero point setting; |
| [9]-[10] | U-phase Current Offset | 2u | Offset obtained from U-phase current calibration; |
| [11]-[12] | V-phase Current Offset | 2u | Offset obtained from V-phase current calibration; |
| [13]-[14] | W-phase Current Offset | 2u | Offset obtained from W-phase current calibration; |
| [15] | Encoder Model | 1u | 0: AS504x_SPI &nbsp;&nbsp;&nbsp;&nbsp; 1: Ma7xx_SPI<br>2: MT6835_SPI &nbsp;&nbsp;&nbsp;&nbsp; 3: TLx5012_SSI<br>4: AS504x_PWM &nbsp;&nbsp; 5: Tamagawa<br>*(This parameter is invalid for driver boards with onboard encoders)* |
| [16] | Change Encoder Direction | 1u | 0: Do not change direction &nbsp;&nbsp; Other: Change direction |
| [17] | Enable Second Encoder | 1u | 0: Disable &nbsp;&nbsp; Other: Enable |
| [18] | Velocity Filter Coefficient | 1u | Value 1~100 represents coefficient 0.01~1; |
| [19] | Device Address | 1u | Address range is 0x01~0xFE (1~254) |
| [20] | RS485 Baud Rate | 1u | 0: 921600 &nbsp;&nbsp; 1: 460800 &nbsp;&nbsp; 2: 115200<br>3: 57600 &nbsp;&nbsp;&nbsp;&nbsp; 4: 38400 &nbsp;&nbsp;&nbsp;&nbsp; 5: 19200<br>6: 9600 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *(Default value is 115200)* |
| [21] | CAN Baud Rate | 1u | 0: 1M &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 1: 500K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 2: 250K<br>3: 125K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 4: 100K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *(Default value is 1M)* |
| [22] | Enable CanOpen | 1u | 0: Can Custom Protocol &nbsp;&nbsp; Other: Canopen Protocol |
| [23]-[24] | Max Bus Voltage | 2u | Unit is 0.01V |
| [25] | Voltage Fault Duration | 1u | If continuous over-voltage duration exceeds this time, report fault; Unit: seconds; |

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [26]-[27] | Max Bus Current | 2u | Unit is 0.01A |
| [28] | Current Fault Duration | 1u | If continuous over-current duration exceeds this time, report fault; Unit: seconds; |
| [29] | Max Temperature | 1u | Max temperature, Unit: ℃; |
| [30] | Temperature Fault Duration | 1u | If continuous over-temperature duration exceeds this time, report fault; Unit: seconds; |
| [31]-[32] | CRC16 Check | 2u | DATA[0]~DATA[30] bytes CRC16 Check |

<br>

➢ **Write and Save User Parameters;** **【Command Code: 0x11】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x11** |
| [4] | Data Packet Length | 1u | 0x10 |
| [5] | Encoder Model | 1u | 0: AS504x_SPI &nbsp;&nbsp;&nbsp;&nbsp; 1: Ma7xx_SPI<br>2: MT6835_SPI &nbsp;&nbsp;&nbsp;&nbsp; 3: TLx5012_SSI<br>4: AS504x_PWM &nbsp;&nbsp; 5: Tamagawa<br>*(This parameter is invalid for driver boards with onboard encoders)* |
| [6] | Change Encoder Direction | 1u | 0: Do not change direction &nbsp;&nbsp; Other: Change direction |
| [7] | Enable Second Encoder | 1u | 0: Disable &nbsp;&nbsp; Other: Enable |
| [8] | Velocity Filter Coefficient | 1u | Value 1~100 represents coefficient 0.01~1 |
| [9] | Device Address | 1u | Address range is 0x01~0xFE (1~254) |
| [10] | RS485 Baud Rate | 1u | 0: 921600 &nbsp;&nbsp; 1: 460800 &nbsp;&nbsp; 2: 115200<br>3: 57600 &nbsp;&nbsp;&nbsp;&nbsp; 4: 38400 &nbsp;&nbsp;&nbsp;&nbsp; 5: 19200<br>6: 9600 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *(Default value is 115200)* |
| [11] | CAN Baud Rate | 1u | 0: 1M &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 1: 500K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 2: 250K<br>3: 125K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 4: 100K &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; *(Default value is 1M)* |
| [12] | Enable CanOpen | 1u | 0: Can Custom Protocol &nbsp;&nbsp; Other: Canopen Protocol |
| [13]-[14] | Max Bus Voltage | 2u | Unit is 0.01V |
| [15] | Voltage Fault Duration | 1u | If continuous over-voltage duration exceeds this time, report fault; Unit: seconds; |
| [16]-[17] | Max Bus Current | 2u | Unit is 0.01A |
| [18] | Current Fault Duration | 1u | If continuous over-current duration exceeds this time, report fault; Unit: seconds; |
| [19] | Max Temperature | 1u | Max temperature, Unit: ℃; |
| [20] | Temperature Fault Duration | 1u | If continuous over-temperature duration exceeds this time, report fault; Unit: seconds; |
| [21]-[22] | CRC16 Check | 2u | DATA[0]~DATA[20] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**
    *   Except for the command code being different, the content of the slave response is consistent with the content of the **0x10** command.

<br>

➢ **Read Motor Hardware Parameters;** **【Command Code: 0x12】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x12** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x12** |
| [4] | Data Packet Length | 1u | 0x1E |
| [5]-[20] | Motor Name | 16b | ASCII characters of the motor name |
| [21] | Motor Pole Pairs | 1u | |
| [22-25] | Phase Resistance | 4f | Phase-to-phase resistance / 2; Unit: Ω; |
| [26-29] | Phase Inductance | 4f | Phase-to-phase inductance / 2; Unit: mH; |
| [30-33] | Torque Constant | 4f | Unit N/m; |
| [34] | Reduction Ratio | 1u | |
| [35]-[36] | CRC16 Check | 2u | DATA[0]~DATA[34] bytes CRC16 Check |

<br>

➢ **Write and Save Motor Hardware Parameters;** **【Command Code: 0x13】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x13** |
| [4] | Data Packet Length | 1u | 0x1E |
| [5]-[20] | Motor Name | 16b | ASCII characters of the motor name |
| [21] | Motor Pole Pairs | 1u | |
| [22-25] | Phase Resistance | 4f | Phase-to-phase resistance / 2; Unit: Ω; |
| [26-29] | Phase Inductance | 4f | Phase-to-phase inductance / 2; Unit: mH; |
| [30-33] | Torque Constant | 4f | Unit N/m; |
| [34] | Reduction Ratio | 1u | |
| [35]-[36] | CRC16 Check | 2u | DATA[0]~DATA[34] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x12** command;

<br>

➢ **Read Motion Control Parameters;** **【Command Code: 0x14】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x14** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x14** |
| [4] | Data Packet Length | 1u | 0x18 |
| [5]-[8] | Position Loop Kp | 4f | |
| [9]-[12] | Position Loop Ki | 4f | |
| [13]-[16] | Position Loop Output Limit | 4u | Position mode max speed, Unit: 0.01 Rpm; |
| [17]-[20] | Velocity Loopnh Kp | 4f | |
| [21]-[24] | Velocity Loop Ki | 4f | |
| [25]-[28] | Velocity Loop Output Limit | 4u | Velocity/Position mode max Q-axis current, Unit: 0.001A |
| [29]-[30] | CRC16 Check | 2u | DATA[0]~DATA[28] bytes CRC16 Check |

<br>

➢ **Write Motion Control Parameters but do not save;** **【Command Code: 0x15】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x15** |
| [4] | Data Packet Length | 1u | 0x18 |
| [5]-[8] | Position Loop Kp | 4f | |
| [9]-[12] | Position Loop Ki | 4f | |
| [13]-[16] | Position Loop Output Limit | 4u | Position mode max speed, Unit: 0.01 Rpm; |
| [17]-[20] | Velocity Loop Kp | 4f | |
| [21]-[24] | Velocity Loop Ki | 4f | |
| [25]-[28] | Velocity Loop Output Limit | 4u | Velocity/Position mode max Q-axis current, Unit: 0.001A |
| [29]-[30] | CRC16 Check | 2u | DATA[0]~DATA[28] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x14** command;

<br>

➢ **Write and Save Motion Control Parameters;** **【Command Code: 0x16】**

*   **Master (Host) sends to Slave (Device)**
    *   Except for the sent command code being different, the content sent by the Master is consistent with the content of the **0x15** command;
*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x14** command;

<br>

➢ **Set Current Position as Origin (Zero Point);** **【Command Code: 0x1D】**

***


Here is the translation for **Page 10**.

This page covers the completion of the Set Origin command (**0x1D**), the Encoder Calibration command (**0x1E**), and the start of the Restore Default Parameters command (**0x1F**).

***


*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x1D** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x1D** |
| [4] | Data Packet Length | 1u | 0x02 |
| [5]-[6] | Mechanical Angle Offset | 2u | |
| [7]-[8] | CRC16 Check | 2u | DATA[0]~DATA[6] bytes CRC16 Check |

<br>

➢ **Encoder Calibration;** Considering the accuracy of the calibration results, the motor must be **unloaded** during the encoder calibration process, and motor rotation cannot be interfered with; if you need to stop the current encoder calibration, you can send the **0x2F** command to stop the operation; **【Command Code: 0x1E】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x1E** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Restore Parameters to Default Values;** (Device address, encoder calibration parameters, and motor hardware parameters remain unchanged) **【Command Code: 0x1F】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x1F** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |

*(Table continues on the next page...)*

Here is the translation for **Page 11**.

This page concludes the **0x1F** command and introduces the control commands: **0x20** (Torque/Current Control), **0x21** (Velocity Control), and starts **0x22** (Absolute Position Control).

***


| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x1F** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

<br>

➢ **Q-axis Current Control (Torque Control);** **【Command Code: 0x20】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x20** |
| [4] | Data Packet Length | 1u | 0x08 |
| [5]-[8] | Target Q-axis Current | 4s | Unit 0.001A; Torque = Target Current * Torque Constant; |
| [9]-[12] | Q-axis Current Ramp | 4u | Unit 0.001A/S; Value of 0 means maximum slope/rate; |
| [13]-[14] | CRC16 Check | 2u | DATA[0]~DATA[12] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Velocity Control;** **【Command Code: 0x21】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x21** |
| [4] | Data Packet Length | 1u | 0x08 |
| [5]-[8] | Target Velocity | 4s | Unit is 0.01 Rpm |
| [9]-[12] | Acceleration | 4u | Unit is 0.01 Rpm/s; Value of 0 means maximum acceleration; |
| [13]-[14] | CRC16 Check | 2u | DATA[0]~DATA[12] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Absolute Position Control;** **【Command Code: 0x22】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x22** |
| [4] | Data Packet Length | 1u | 0x04 |
| [5]-[8] | Absolute Position | 4s | Unit is Count; One rotation is 16384 Count |
| [9]-[10] | CRC16 Check | 2u | DATA[0]~DATA[8] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Relative Position Control;** **【Command Code: 0x23】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x23** |
| [4] | Data Packet Length | 1u | 0x04 |
| [5]-[8] | Relative Position Value | 4s | Unit is Count; One rotation is 16384 Count |
| [9]-[10] | CRC16 Check | 2u | DATA[0]~DATA[8] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Return to Origin via Shortest Distance;** **【Command Code: 0x24】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x24** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

<br>

➢ **Holding Brake Control Switch Output Control;** **【Command Code: 0x2E】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x2E** |
| [4] | Data Packet Length | 1u | 0x01 |
| [5] | Operation Type | 1u | 0x00: Switch Open (Disconnect)<br>0x01: Switch Closed (Connect)<br>0xFF: Read Status |
| [6]-[7] | CRC16 Check | 2u | DATA[0]~DATA[5] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**

| Index | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAC |

*(Table continues on the next page...)*

Here is the translation for **Page 13** and **Page 14**.

**Page 13** concludes the list of control commands (finishing Holding Brake control and adding Motor Disable). **Page 14** begins the section on **Communication Examples**, providing detailed breakdowns of how to parse the Hex data.

***


| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [1] | Packet Sequence | 1u | 0x00-0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x2E** |
| [4] | Data Packet Length | 1u | 0x01 |
| [5] | Holding Brake Switch Status | 1u | 0x00: Switch Open (Disconnect) &nbsp;&nbsp; 0x01: Switch Closed (Connect) |
| [6]-[7] | CRC16 Check | 2u | DATA[0]~DATA[5] bytes CRC16 Check |

<br>

➢ **Motor Disable OFF;** **【Command Code: 0x2F】**

*   **Master (Host) sends to Slave (Device)**

| Index | Field Name | Type | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| [0] | Protocol Header | 1u | 0xAE |
| [1] | Packet Sequence | 1u | 0x00~0xFF |
| [2] | Device Address | 1u | 0x01~0xFE |
| [3] | Command Code | 1u | **0x2F** |
| [4] | Data Packet Length | 1u | 0x00 |
| [5]-[6] | CRC16 Check | 2u | DATA[0]~DATA[4] bytes CRC16 Check |

*   **Slave (Device) replies to Master Controller (Host)**
    *   Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0x0B** command;

***

### Custom Communication Protocol Communication Examples:

**The command format for the RS485_V3.x version protocol communication data packet is as follows:**

| Field Name | Byte Count | Description |
| :--- | :--- | :--- |
| **Protocol Header** | 1Byte | The protocol header sent by the Master is **0xAE**; the protocol header replied by the Slave is **0xAC**; |
| **Packet Sequence** | 1Byte | The packet sequence number replied by the Slave is consistent with the packet sequence number sent by the Master to the Slave; |
| **Device Address** | 1Byte | Address range is 0-0xFE; 0x00 is Broadcast Address; 0xFF is Public Address;<br>**Broadcast Address:** Slaves can receive and execute instructions but do not reply to the Master;<br>**Public Address:** Slaves can receive and execute instructions but do reply to the Master; Determined by RS485 bus characteristics, when multiple motors are connected to the bus, the Public Address cannot be used; |
| **Command Code** | 1Byte | Different commands have different command codes; |
| **Data Packet Length** | 1Byte | **[Data Field]** Byte quantity (can be 0); |
| **Data Field** | 0-248(Bytes) | Data attached to the command code; |
| **CRC16 Check** | 2Bytes | CRC16_MODBUS check value from **[Protocol Header]** to **[Data Field]**; For checksum calculation, refer to the resource package "CRC16_MODBUS Check C Language Source Code" |

<br>

**1. [0x0B Command] - Read single-turn absolute angle, multi-turn absolute angle, velocity, Q-axis current, etc. real-time information;** Assuming the Slave device address is 1, the communication data frame is as follows:

**Master Controller sends data frame (Hex):** (Data packet length is 0, Data field is empty)
`AE 00 01 0B 00 9B 28`

**Slave replies data frame (Hex):**
`AC 00 01 0B 16 27 39 27 39 19 00 1E C8 00 00 19 00 00 00 94 0C 04 00 24 03 01 00 3B DD`
*(Markers ① through ⑩ correspond to the table below)*

**Parsing the Slave Reply Data Field based on 0x0B Protocol Content:**

| No. | Field Name | Hex | Dec | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- | :--- |
| ① | Single-turn absolute angle | 0x3927 | 14631 | 14631*(360/16384) = **321.48°** |
| ② | Multi-turn absolute angle | 0x00193927 | 1653031 | 1653031*(360/16384) = **36321.48°** |
| ③ | Mechanical Velocity | 0x0000C81E | 51230 | 51230*0.01 = **512.3 Rpm** |
| ④ | Q-axis Current | 0x00000019 | 25 | 25*0.001 = **0.025A** |
| ⑤ | Bus Voltage | 0x0C94 | 3220 | 3220*0.01 = **32.2V** |
| ⑥ | Bus Current | 0x0004 | 4 | 4*0.01 = **0.04A** |
| ⑦ | Working Temperature | 0x24 | 36 | **36℃** |
| ⑧ | Running Status | 0x03 | 3 | **Velocity Control Mode** |
| ⑨ | Motor Status | 0x01 | 1 | **Enable** |
| ⑩ | Fault Code | 0x00 | 0 | **No Fault** |

***

Here is the translation for **Page 15**.

This page covers **Example 2** (Clear Faults), **Example 3** (Torque Control), and the beginning of **Example 4** (Velocity Control).

***

# Page 15

**2. [0x0F Command] - Clear Faults;** Assuming the Slave address is 1, the communication data frame is as follows:

**Master Controller sends data frame (Hex):**
`AE 00 01 0F 00 99 E8`

**Slave replies data frame (Hex):**
`AC 00 01 0F 01 00 28 18`
*(Marker ① corresponds to the table below)*

**Parsing the Master Controller received data field based on 0x0F Protocol Content:**

| No. | Field Name | 16-bit (Hex) | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- |
| ① | Fault Code | 0x00 | [Bit0]: Voltage fault; [Bit1]: Current fault; [Bit2]: Temperature fault; [Bit3]: Encoder fault; [Bit6]: Hardware fault; [Bit7]: Software fault; **0x00 indicates no fault;** |

<br>

**3. [0x20 Command] - Torque Control, Target Q-axis Current 1A;** Assuming the Slave address is 1, the communication data frame is as follows:

**Master Controller sends data frame (Hex):**
`AE 00 01 20 08 E8 03 00 00 00 00 00 00 CB 7C`
*(Markers ① and ② correspond to the table below)*

**Slave replies data frame (Hex):** (Reply content format is consistent with 0x0B command)
`AC 00 01 20 16 8B 12 8B 92 5C 00 00 00 00 00 A3 FF FF FF 7D 09 01 00 2C 02 01 00 43 D3`

**Parsing the Master Controller sent data field based on 0x20 Protocol Content:**

| No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning | No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| ① | Q-axis Current | 0x00 00 03 E8 | 1000 | Q-axis Current = 1000*0.001 = **1A** | ② | Current Ramp | 0x00000000 | 0 | Q-axis Current Ramp 0, **Max Ramp** |

<br>

**4. [0x21 Command] - Velocity Control, Target Velocity 100Rpm;** Assuming the Slave device address is 1, the communication data frame is as follows:

**Master Controller sends data frame (Hex):** (Data packet length is 0*, data field is empty) *(Note: This text in parenthesis seems to be a copy-paste error in the original Chinese PDF, as 0x21 clearly has a data length of 8 bytes in the example below)*
`AE 00 01 21 08 10 27 00 00 00 00 00 00 F0 59`
*(Markers ① and ② correspond to the table on the next page)*

**Slave replies data frame (Hex):** (Reply content format is consistent with 0x0B command)


This page completes **Example 4** (Velocity Control) and covers **Example 5** (Absolute Position Control) and **Example 6** (Relative Position Control).

***


`AC 00 01 21 16 33 3E 33 3E 00 00 91 27 00 00 3C 00 00 00 7E 09 02 00 24 03 01 00 1E 46`

**Parsing the Master Controller sent data field based on 0x21 Protocol Content:**

| No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning | No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| ① | Velocity | 0x00 00 27 10 | 10000 | 10000*0.01= **100 Rpm** | ② | Acceleration | 0x0000 | 0 | Acceleration 0, **Max Acceleration** |

<br>

**5. [0x22 Command] - Absolute Position Control to Target Position 16384 count; Assuming Slave device address is 1, communication data frame is as follows:**

**Master Controller sends data frame (Hex):** (Data packet length is 4*) *[Note: The Chinese text says length 0/empty field erroneously, but the hex code shows length 04 and data is present]*
`AE 00 01 22 04 00 40 00 00 58 C1`
*(Marker ① corresponds to the table below)*

**Slave replies data frame (Hex):** (Reply content format is consistent with 0x0B command)
`AC 00 01 22 16 33 3E 33 3E 00 00 91 27 00 00 3C 00 00 00 7E 09 02 00 24 03 01 00 FA B9`

**Parsing the Master Controller sent data field based on 0x22 Protocol Content:**

| No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- | :--- |
| ① | Absolute Position | 0x 00 40 00 00 | 16384 | 16384*(360/16384) = 360°, motor rotates to multi-turn absolute position 16384 count position; |

<br>

**6. [0x23 Command] - Relative Position Control rotate 90 degrees; Assuming Slave address is 1, communication data frame is as follows:**

**Master Controller sends data frame (Hex):** (Count value corresponding to 90 degrees is 90/360*16384 = 4096, converted to 16-bit Hex is 0x00001000, protocol uses Little-Endian byte order, filled content is 00 10 00 00)
`AE 00 01 23 04 00 10 00 00 59 01`
*(Marker ① corresponds to the table below)*

**Slave replies data frame (Hex):** (Reply content format is consistent with 0x0B command)
`AC 00 01 23 16 F6 22 F6 E2 73 00 00 00 00 00 00 00 00 00 94 0C 00 00 23 04 01 00 40 67`

**Parsing the Master Controller sent data field based on 0x23 Protocol Content:**

| No. | Field Name | 16-bit (Hex) | 10-bit (Dec) | Corresponding Parameter Value / Meaning |
| :--- | :--- | :--- | :--- | :--- |
| ① | Relative Position | 0x 00 10 00 00 | 4096 | 4096*(360/16384) = 90°, motor rotates 90 degrees in positive direction based on current position; |

Here is the translation for **Page 17** (the final page).

This page covers **Example 7** (Motor Disable).

***


**7. [0x2F Command] Motor Disable; Assuming Slave address is 1, the communication data frame is as follows:**

**Master Controller sends data frame (Hex):**
`AE 00 01 2F 00 80 28`

**Slave replies data frame (Hex):** (Reply content format is consistent with 0x0B command)
`AC 00 01 2F 16 9B 3C 9B BC E5 04 00 00 00 00 00 00 00 00 7D 09 01 00 2E 00 00 00 1A 6A`

***

