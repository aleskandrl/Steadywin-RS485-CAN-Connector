cmake_minimum_required(VERSION 3.10)
project(steadywin_motor_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(steadywin_connect/include)

# Source files for the library
set(LIB_SOURCES
    steadywin_connect/src/protocol/steadywin_protocol_rs485.cpp
    steadywin_connect/src/protocol/steadywin_protocol_can.cpp
    steadywin_connect/src/protocol/steadywin_protocol.cpp
    steadywin_connect/src/hal/serial_can_port.cpp
    steadywin_connect/src/hal/gs_usb_can_port.cpp
    steadywin_connect/src/core/steadywin_motor.cpp
)

# Add Windows-specific implementation
if(WIN32)
    list(APPEND LIB_SOURCES steadywin_connect/src/hal/windows_serial_port.cpp)
endif()

# Create the library
add_library(steadywin STATIC ${LIB_SOURCES})

# --- Main Executable ---
add_executable(steadywin_connector src/main.cpp)
target_link_libraries(steadywin_connector steadywin)

# --- Test Executables ---
if(WIN32)
    add_executable(candlelight_test tests/candlelight_test.cpp)
    target_link_libraries(candlelight_test setupapi winusb)
endif()

if(WIN32)
    # Necessary libraries for Windows (both MSVC and MinGW/GCC)
    # We need setupapi for serial port enumeration and winusb for CandleLight
    target_link_libraries(steadywin_connector setupapi ws2_32 winusb)
    
    if(MINGW)
        # MinGW needs explicit link to some standard libs sometimes
        target_link_libraries(steadywin_connector -lsetupapi -lws2_32 -lwinusb)
        if(TARGET candlelight_test)
            target_link_libraries(candlelight_test -lsetupapi -lwinusb)
        endif()
    endif()
endif()
