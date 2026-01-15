

# Custom CAN Communication Protocol_V3.07b0

### Protocol Version:

*   **Rev.3.01b0 – 01.06.2024:** Initial version;
*   **Rev.3.02b0 – 22.07.2024:** Optimized system reset protocol;
*   **Rev.3.03b0 – 01.08.2024:** Added PID parameter configuration commands for position loop and velocity loop control;
*   **Rev.3.04b0 – 21.11.2024:** Added holding brake output control; added encoder faults to fault codes;
*   **Rev.3.05b0 – 18.03.2025:** Corrected descriptive errors in the document;
    *   Enabled reception and response to commands at address `(0x100 | Dev_addr)`;
    *   Added active reporting of status information corresponding to the 0xAE command upon fault detection;
    *   Improved communication examples (details at the end of the file);
*   **Rev.3.05b1 – 03.04.2025:** Optimized document content and corrected erroneous descriptions;
*   **Rev.3.06b0 – 23.04.2025:** Added MIT protocol type motion control mode;
*   **Rev.3.07b0 – 18.05.2025:** Supplemented description for "Set current position as origin" command;
    *   Added content for command code 0xA4.

<br>

### Protocol Introduction:

The CAN interface default communication baud rate is **1MHz**. It can be configured via the host computer to the following: 1MHz, 500KHz, 250KHz, 125KHz, and 100KHz. All CAN messages under the current protocol use the **Data Frame** format and **Standard Frame** format, and the byte order follows **Little-Endian** mode.

When the product ships, the default **Device Address (Dev_addr)** is 0x01. This address can be adjusted via the host computer to any value between 1 and 254. The system defines two special addresses: **0x00** as the Broadcast Address (all slaves execute the command but do not reply to the host); **255 (i.e., 0xFF)** as the Public Address (all slaves respond to commands at this address and reply, with the reply ID being the device address).

To distinguish whether a data frame on the bus is sent by the Master or replied by a Slave, a mechanism is designed. The Slave not only directly responds to commands matching its `Dev_addr`, but also recognizes and responds to commands with ID `(0x100 | Dev_addr)`. Regardless of the case, the ID of the response frame from the Slave to the Master is always `Dev_addr`. For example, if the Slave device address `Dev_addr` is 0x01, and the Master sends a data frame with ID 0x101, the Slave replies with a frame ID of 0x01. This allows the communication direction on the CAN bus to be identified by the Frame ID.

Additionally, the current address of the device can be identified by the flashing pattern of the green LED on the driver board:
*   Device Address 1: `[ __ -__-__ ]`
*   Device Address 2: `[ __-_-__-_-__]`
*   Device Address 3: `[ __-_-_-__-_-_-__]`

**Data Type Description:**
*   **1u** – 1 Unsigned Byte;
*   **1s** – 1 Signed Byte;
*   **2u** – 2 Unsigned Bytes (Little-Endian byte order);
*   **2s** – 2 Signed Bytes (Little-Endian byte order);
*   **4u** – 4 Unsigned Bytes (Little-Endian byte order);
*   **4s** – 4 Signed Bytes (Little-Endian byte order);
*   **4f** – Single-precision floating point number (Little-Endian byte order);
*   **b** – Byte count;
*   **bit** – Indicates a bit; 1 Byte = 8 bits;

<br>

### Supported CAN Custom Control Commands are as follows:

| Category | Command Code | Command Function Description |
| :--- | :---: | :--- |
| **System** | **0x00** | Reboot Slave. After the Master sends this command, the Slave reboots immediately and does not reply to the Master; |
| | **0xA0** | Read Boot, software, hardware, and Custom CAN protocol versions; |
| | **0xA1** | Read real-time Q-axis current; |
| | **0xA2** | Read real-time rotational velocity; |
| | **0xA3** | Read real-time single-turn absolute angle and multi-turn absolute angle; |
| | **0xA4** | Read real-time temperature, Q-axis current, velocity, and single-turn absolute angle; |
| | **0xAE** | Read real-time bus voltage, bus current, working temperature, running mode, and fault code status information;<br>Once the Slave detects a fault, it will actively report real-time status information at a 200ms interval; |
| | **0xAF** | Clear faults; |
| **Parameters** | **0xB0** | Read motor pole pairs, torque constant, and reduction ratio; |
| | **0xB1** | Set current position as origin; |
| | **0xB2** | Set max rotation speed for position mode (not saved on power off); |
| | **0xB3** | Set max Q-axis current for position or velocity mode (not saved on power off); |
| | **0xB4** | Set Q-axis current slope for Q-axis current control mode (not saved on power off); |
| | **0xB5** | Set acceleration for velocity control mode (not saved on power off); |
| | **0xB6** | Read or set Position Control Loop Kp (not saved on power off); |
| | **0xB7** | Read or set Position Control Loop Ki (not saved on power off); |
| | **0xB8** | Read or set Velocity Control Loop Kp (not saved on power off); |
| | **0xB9** | Read or set Velocity Control Loop Ki (not saved on power off); |
| **Control** | **0xC0** | Q-axis current control; (Torque = Torque Constant * Q-axis Current); |
| | **0xC1** | Velocity control; |
| | **0xC2** | Absolute position control; |
| | **0xC3** | Relative position control; |
| | **0xC4** | Motor returns to the set origin via the shortest distance; rotation angle not greater than 180 degrees; |
| | **0xCE** | Holding brake switch output control; |
| | **0xCF** | Turn off motor output. The motor enters a free state and is uncontrolled; (This is the state after motor power-on); |
| **Motion Control (MIT Protocol)** | **0xF0** | Read and configure Pos_Max, Vel_Max, T_Max in Motion Control mode; |
| | **0xF1** | Read real-time position, velocity, torque, and status information under Motion Control mode;<br>Motion control command, no command code; Frame Identifier (StdID) highest bit Bit[10] set to 1; |

<br>

---

### Command Details

#### ➢ Reboot Slave
After the Master sends this command packet, the Slave reboots immediately and does not reply to the Master; **【Command Code: 0x00】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x08` |
| [0] | Command Code | 1u | `0x00` |
| [1]-[7] | Attached 7bytes Data | 7b | `0xFF 0x00 0xFF 0x00 0xFF 0x00 0xFF` |

<br>

#### ➢ Read Boot, software, hardware, and Custom CAN protocol versions; **【Command Code: 0xA0】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xA0` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x08` |
| [0] | Command Code | 1u | `0xA0` |
| [1]-[2] | Boot Software Ver | 2u | |
| [3]-[4] | App Software Ver | 2u | |
| [5]-[6] | Hardware Version | 2u | Driver board hardware code; |
| [7] | CAN Custom Ver | 1u | Current Custom CAN protocol version; |

<br>

#### ➢ Read Real-time Q-axis Current; **【Command Code: 0xA1】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xA1` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xA1` |
| [1]-[4] | Q-axis Current | 4s | Unit 0.001A; (Torque = Torque Constant * Q-axis Current) |

<br>

#### ➢ Read Real-time Rotation Velocity; **【Command Code: 0xA2】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xA2` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xA2` |
| [1]-[4] | Rotation Velocity | 4s | Unit 0.01 Rpm; |

<br>

#### ➢ Read Real-time Single-turn Absolute Angle, Multi-turn Absolute Angle; **【Command Code: 0xA3】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xA3` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x07` |
| [0] | Command Code | 1u | `0xA3` |
| [1]-[2] | Single-turn Abs Angle | 2u | Angle° = value*(360/16384); |
| [3]-[6] | Multi-turn Abs Angle | 4s | Total Angle° = value*(360/16384); |

<br>

#### ➢ Read Real-time Temperature, Q-axis Current, Velocity, Single-turn Absolute Angle; **【Command Code: 0xA4】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xA4` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x08` |
| [0] | Command Code | 1u | `0xA4` |
| [1] | Working Temp | 1u | Unit ℃; |
| [2]-[3] | Q-axis Current | 2s | Unit 0.001A; (Torque = Torque Constant * Q-axis Current) |
| [4]-[5] | Rotation Velocity | 2s | Unit 0.01 Rpm; |
| [6]-[7] | Single-turn Abs Angle | 2u | Angle° = value*(360/16384); |

<br>

#### ➢ Read Bus Voltage, Bus Current, Working Temperature, Running Mode, and Fault Code Status;
When Slave detects a fault, it will report real-time status at 200ms intervals; **【Command Code: 0xAE】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xAE` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x08` |
| [0] | Command Code | 1u | `0xAE` |
| [1]-[2] | Bus Voltage | 2u | Unit 0.01V; |
| [3]-[4] | Bus Current | 2u | Unit 0.01A; |
| [5] | Working Temperature | 1u | Unit ℃; |
| [6] | Running Mode | 1u | 0: Closed (Off) state; 1: Voltage control;<br>2: Q-axis current control; 3: Velocity control;<br>4: Position control; |
| [7] | Fault Code | 1u | [Bit0]: Voltage fault; [Bit1]: Current fault;<br>[Bit2]: Temperature fault; [Bit3]: Encoder fault;<br>[Bit6]: Hardware fault; [Bit7]: Software fault;<br>Bitn represents the nth bit of the byte; Bit0 represents the 0th bit; |

<br>

#### ➢ Clear Faults; **【Command Code: 0xAF】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xAF` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x02` |
| [0] | Command Code | 1u | `0xAF` |
| [1] | Current Fault | 1u | [Bit0]: Voltage fault; [Bit1]: Current fault;<br>[Bit2]: Temperature fault; [Bit3]: Encoder fault;<br>[Bit6]: Hardware fault; [Bit7]: Software fault;<br>Bitn represents the nth bit of the byte; Bit0 represents the 0th bit; |

<br>

#### ➢ Read Motor Pole Pairs, Torque Constant, Reduction Ratio; **【Command Code: 0xB0】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xB0` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x07` |
| [0] | Command Code | 1u | `0xB0` |
| [1] | Motor Pole Pairs | 1u | |
| [2]-[5] | Torque Constant | 4f | Unit is Nm/A; |
| [6] | Reduction Ratio | 1u | |

<br>

#### ➢ Set Current Position as Origin;
The motor's single-turn absolute value origin is saved to the driver board and is not lost on power off; If the system has enabled the second encoder, the execution time for this operation is longer, and the interval for the Slave to reply to the Master is approximately 35ms; **【Command Code: 0xB1】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xB1` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x03` |
| [0] | Command Code | 1u | `0xB1` |
| [1]-[2] | Mechanical Angle Offset | 2u | |

<br>

#### ➢ Set Max Rotation Speed for Position Mode, not saved on power off; **【Command Code: 0xB2】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB2` |
| [1]-[4] | Position Loop Output Limit | 4u | Max speed in position mode, Unit 0.01 Rpm; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB2` |
| [1]-[4] | Position Loop Output Limit | 4u | Max speed in position mode, Unit 0.01 Rpm; |

<br>

#### ➢ Set Max Q-axis Current for Position or Velocity Mode, not saved on power off; **【Command Code: 0xB3】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB3` |
| [1]-[4] | Velocity Loop Output Limit | 4u | Max Q-axis current in Velocity/Position mode, Unit 0.001A; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB3` |
| [1]-[4] | Velocity Loop Output Limit | 4u | Max Q-axis current in Velocity/Position mode, Unit 0.001A; |

<br>

#### ➢ Set Q-axis Current Slope for Q-axis Current Control Mode, not saved on power off; **【Command Code: 0xB4】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB4` |
| [1]-[4] | Q-axis Current Slope | 4u | Current output rate, Unit 0.001A/s; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB4` |
| [1]-[4] | Q-axis Current Slope | 4u | Current output rate, Unit 0.001A/s; |

<br>

#### ➢ Set Acceleration for Velocity Control Mode, not saved on power off; **【Command Code: 0xB5】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB5` |
| [1]-[4] | Acceleration | 4u | Unit is 0.01 Rpm/s; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB5` |
| [1]-[4] | Acceleration | 4u | Unit is 0.01 Rpm/s; |

<br>

#### ➢ Read or Set Position Control Loop Kp, not saved on power off; **【Command Code: 0xB6】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `[0x01, Read Params]`, `[0x05, Configure Params]` |
| [0] | Command Code | 1u | `0xB6` |
| [1]-[4] | Position Control Loop Kp | 4f | If DLC is 1, this content is empty |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB6` |
| [1]-[4] | Position Control Loop Kp | 4f | |

<br>

#### ➢ Read or Set Position Control Loop Ki, not saved on power off; **【Command Code: 0xB7】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `[0x01, Read Params]`, `[0x05, Configure Params]` |
| [0] | Command Code | 1u | `0xB7` |
| [1]-[4] | Position Control Loop Ki | 4f | If DLC is 1, this content is empty |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB7` |
| [1]-[4] | Position Control Loop Ki | 4f | |

<br>

#### ➢ Read or Set Velocity Control Loop Kp, not saved on power off; **【Command Code: 0xB8】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `[0x01, Read Params]`, `[0x05, Configure Params]` |
| [0] | Command Code | 1u | `0xB8` |
| [1]-[4] | Velocity Control Loop Kp | 4f | If DLC is 1, this content is empty |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB8` |
| [1]-[4] | Velocity Control Loop Kp | 4f | |

<br>

#### ➢ Read or Set Velocity Control Loop Ki, not saved on power off; **【Command Code: 0xB9】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `[0x01, Read Params]`, `[0x05, Configure Params]` |
| [0] | Command Code | 1u | `0xB9` |
| [1]-[4] | Velocity Control Loop Ki | 4f | If DLC is 1, this content is empty |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xB9` |
| [1]-[4] | Velocity Control Loop Ki | 4f | |

<br>

#### ➢ Q-axis Current Control; **【Command Code: 0xC0】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xC0` |
| [1]-[4] | Q-axis Current | 4s | Unit 0.001A; (Torque = Q-axis Current * Torque Constant); |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xA1** command;

<br>

#### ➢ Velocity Control; **【Command Code: 0xC1】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xC1` |
| [1]-[4] | Rotation Velocity | 4s | Unit is 0.01 Rpm; |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xA2** command;

<br>

#### ➢ Absolute Position Control; **【Command Code: 0xC2】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xC2` |
| [1]-[4] | Absolute Position | 4s | Unit is Count; One motor rotation is 16384 Count; |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xA3** command;

<br>

#### ➢ Relative Position Control; **【Command Code: 0xC3】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x05` |
| [0] | Command Code | 1u | `0xC3` |
| [1]-[4] | Relative Position | 4s | Unit is Count; One motor rotation is 16384 Count; |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xA3** command;

#### ➢ Return to Origin via Shortest Distance; **【Command Code: 0xC4】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xC4` |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xA3** command;

<br>

#### ➢ Holding Brake Switch Output Control; **【Command Code: 0xCE】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x02` |
| [0] | Command Code | 1u | `0xCE` |
| [1] | Operation Type | 1u | 0x00: Switch Open (Disconnect); 0x01: Switch Closed (Connect);<br>0xFF: Read Status; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x02` |
| [0] | Command Code | 1u | `0xCE` |
| [1] | Brake Switch Status | 1u | 0x00: Switch Open (Disconnect); 0x01: Switch Closed (Connect); |

<br>

#### ➢ Turn Off Motor Output, Motor Enters Free State Uncontrolled (State after power-on); **【Command Code: 0xCF】**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xCF` |

**Slave (Device) replies to Master Controller (Host):**
Except for the reply command code being different, the content of the slave reply is consistent with the content of the **0xAE** command;

---

### Motion Control Mode: MIT Type Protocol Control Method

To be compatible with motors of different specifications, the protocol supports configuring maximum values for Position, Velocity, and Torque based on the current motor's actual parameters or usage requirements. It is important to note that in the current motion control protocol, the units for Position, Velocity, and Torque differ from those used previously:

1.  **Position Unit:** Radians (rad), where 2π rad corresponds to 360°;
2.  **Velocity Unit:** Radians per second (rad/s), where 2π rad/s corresponds to 60 RPM;
3.  **Torque Unit:** Newton-meters (Nm), requires correct configuration of motor torque constant via the host computer;

**Detailed Protocol Content for Motion Control Mode:**

#### ➢ Read and Configure Pos_Max, Vel_Max, T_Max in Motion Control Mode;
Data follows Little-Endian byte order; Configured parameters are saved to the driver board and not lost on power off; **(Command Code: 0xF0)**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `[0x01, Read Params]`, `[0x07, Configure Params]` |
| [0] | Command Code | 1u | `0xF0` |
| [1]-[2] | Pos_Max | 2u | Configure Motion Control Mode Max Position, Unit 0.1rad; |
| [3]-[4] | Vel_Max | 2u | Configure Motion Control Mode Max Velocity, Unit 0.01rad/s; |
| [5]-[6] | T_Max | 2u | Configure Motion Control Mode Max Torque, Unit 0.01Nm; |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x07` |
| [0] | Command Code | 1u | `0xF0` |
| [1]-[2] | Pos_Max | 2u | Motion Control Mode Max Position, Unit 0.1rad;<br>Default 955, i.e., Pos_Max is 95.5rad; |
| [3]-[4] | Vel_Max | 2u | Motion Control Mode Max Velocity, Unit 0.01rad/s;<br>Default 4500, i.e., Vel_Max is 45.00rad/s; |
| [5]-[6] | T_Max | 2u | Motion Control Mode Max Torque, Unit 0.01Nm;<br>Default 1800, i.e., T_Max is 18.00Nm; |

#### ➢ Read Real-time Position, Velocity, Torque, and Status Information under Motion Control Mode; **(Command Code: 0xF1)**

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr`, `(0x100\|Dev_addr)`, `0x00`, `0xFF` |
| DLC | Frame Length | | `0x01` |
| [0] | Command Code | 1u | `0xF1` |

**Slave (Device) replies to Master Controller (Host):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `Dev_addr` |
| DLC | Frame Length | | `0x07` |
| [0] | Command Code | 1u | `0xF1` |
| [1] | Mechanical Position | 16bit | Byte[1] is High 8 bits, Byte[2] is Low 8 bits;<br>[0~65535] corresponds to (-Pos_Max ~ Pos_Max); |
| [2] | | | |
| [3] | Mechanical Velocity | 12bit | Byte[3] is High 8 bits, Byte[4]-Bit[7:4] is Low 4 bits;<br>[0~4095] corresponds to (-Vel_Max ~ Vel_Max); |
| [4] | Bit[7:4] | | |
| [4] | Bit[3:0] | Torque | 12bit |
| [5] | | | Byte[4]-Bit[3:0] is High 4 bits, Byte[5] is Low 8 bits;<br>[0~4095] corresponds to (-T_Max ~ T_Max); |
| [6] | Status Info | 1u | [Bit0]: 1 indicates currently in Motion Control Mode;<br>[Bit1]: 1 indicates system has a fault; |

<br>

#### ➢ Motion Control Command;
Since the current command data frame sent by the host does not contain a command code, to enable the slave to accurately identify this data frame as a Motion Control Mode command, the host sending the data frame must set the **Highest Bit (Bit[10])** of the `StdID` to **1** (If device address `Dev_addr` is 1, `StdID` is set to 0x401 or 0x501); Upon receiving the current command, the slave will immediately switch to Motion Control Mode and execute the command; To exit Motion Control Mode, send command code **0xCF** (see above description); To set current position as origin, send **0xB1** command (see above description);

**Master Controller (Host) sends to Slave (Device):**

| CAN Frame | Field Name | Byte | Content Description (Data) |
| :--- | :--- | :--- | :--- |
| StdID | Device Address | 11bit | `(0x400\|Dev_addr)`, `(0x400\|(0x100\|Dev_addr))`,<br>`(0x400\|0x00)`, `(0x400\|0xFF)` |
| DLC | Frame Length | | `0x08` |
| [0] | Target Position | 16bit | Byte[0] is High 8 bits, Byte[1] is Low 8 bits;<br>[0~65535] corresponds to (-Pos_Max ~ Pos_Max); |
| [1] | | | |
| [2] | Target Velocity | 12bit | Byte[2] is High 8 bits, Byte[3]-Bit[7:4] is Low 4 bits;<br>[0~4095] corresponds to (-Vel_Max ~ Vel_Max); |
| [3] | Bit[7:4] | | |
| [3] | Bit[3:0] | Position Gain Kp Value | 12bit | Byte[3]-Bit[3:0] is High 4 bits, Byte[4] is Low 8 bits;<br>[0~4095] corresponds to [0~500]; |
| [4] | | | |
| [5] | Velocity Gain Kd Value | 12bit | Byte[5] is High 8 bits, Byte[6]-Bit[7:4] is Low 4 bits;<br>[0~4095] corresponds to [0~5]; |
| [6] | Bit[7:4] | | |
| [6] | Bit[3:0] | Target Torque | 12bit | Byte[6]-Bit[3:0] is High 4 bits, Byte[7] is Low 8 bits;<br>[0~4095] corresponds to (-T_Max ~ T_Max); |
| [7] | | | |

**Slave (Device) replies to Master Controller (Host):**
The content of the slave response is consistent with the content of the **0xF1** command;

---

### CAN Protocol Communication Packet Command Examples:
*(CAN messages use Data Frame format and Standard Frame format, byte order follows Little-Endian mode)*

**[0xAF Command] - Clear Faults**

*   Master sends data (HEX): `AF`; DLC Length is 1;
    *   `AF`
*   Master receives data (HEX): `AF 00`; DLC Length is 2;
    *   `AF 00`

Parsing Master received data field based on 0xAF protocol content:
*   **0x00**: Fault Code; 0x00 means No Fault, other fault codes see manual [0xAF] command content;

<br>

**[0xC0 Command] - Torque Mode Control, Target Q-axis Current is 1A;**

*   Master sends data (HEX): `C0 E8 03 00 00`; 0x000003E8 Decimal is 1000, 1000*0.001=1A; DLC Length is 5;
    *   `C0 E8 03 00 00`
*   Master receives data (HEX): `C0 00 00 03 F5`; DLC Length is 5;
    *   `C0 F5 03 00 00`

Parsing Master received data field based on 0xC0 protocol content:
*   **0x000003F5**: Current Torque; 0x000003F5 Decimal is 1013; 1013*0.001=1.013A;

<br>

**[0xC1 Command] - Velocity Mode Control, Target Velocity 100Rpm;**

*   Master sends data (HEX): `C1 10 27 00 00`, 0x00002710 Decimal is 10000, 10000*0.01=100Rpm; DLC Length is 5;
    *   `C1 10 27 00 00`
*   Master receives data (HEX): `0xC100002738`; DLC Length is 5;
    *   `C1 38 27 00 00`

Parsing Master received data field based on 0xC1 protocol content:
*   **0x00002738**: Current Velocity; 0x00002738 Decimal is 10040; 10040*0.01=100.4Rpm;

<br>

**[0xC2 Command] - Absolute Position Mode Control, Absolute Target Position is 16384 Count (Multi-turn absolute angle is 1 turn position);**

*   Master sends data (HEX): `C2 00 40 00 00`, 0x00004000 Decimal is 16384; Motor rotates to 360° position; DLC Length is 5;
    *   `C2 00 40 00 00`
*   Master receives data (HEX): `C2 00 00 00 00`; DLC Length is 7;
    *   `C2 00 00 00 00 00 00`

Parsing Master received data field based on 0xC2 protocol content:
*   **[1]-[2]:0x0000**: Motor Current Single-turn Absolute Position; Angle 0°;
*   **[3]-[6]:0x0000**: Motor Current Multi-turn Absolute Position; Angle 0°;

<br>

**[0xC3 Command] - Relative Position Mode Control, Motor Rotates 90° in Positive Direction Relative to Current Position;**

*   Master sends data (HEX): `C3 00 10 00 00`, 0x00001000 Decimal is 4096, 4096*(360/16384) = 90°; DLC Length is 5;
    *   `C3 00 10 00 00`
*   Master receives data (HEX): `C3 00 40 00 00`; DLC Length is 7;
    *   `C3 00 40 00 40 00 00`

Parsing Master received data field based on 0xC3 protocol content:
*   **[1]-[2]:0x4000**: Motor Current Single-turn Absolute Position; 4000 Decimal is 16384, Angle 16384*(360/16384) = 360°;
*   **[3]-[6]:0x00004000**: Motor Current Multi-turn Absolute Position; 4000 Decimal is 16384, Angle 16384*(360/16384) = 360°;

<br>

**[0xCF Command] - Motor Disable**

*   Master sends data (HEX): DLC Length is 1;
    *   `CF`
*   Master receives data (HEX): `CF 7C 09 01 00 26 00 00`; DLC Length is 8;
    *   `CF 7C 09 01 00 26 00 00`
    *   `①  ②  ③  ④  ⑤`

Parsing Master received data field based on 0xCF protocol content:
*   **① 0x097C**: 0x097C Decimal is 2428, Bus Voltage is 2428*0.01=24.28V;
*   **② 0x0001**: 0x0001 Decimal is 1, Bus Current is 1*0.01=0.001A;
*   **③ 0x26**: 0x26 Decimal is 38, Motor Temperature is 38°C;
*   **④ 0x00**: Running Mode; 0x00 is Closed (Off) state, see manual for details;
*   **⑤ 0x00**: Fault Code; 0x00 is No Fault, see manual for details;