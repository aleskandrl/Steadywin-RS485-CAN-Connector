cmake_minimum_required(VERSION 3.10)
project(steadywin_CAN_RS485_control)

# --- Standard Options ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Add steadywin_connect library ---
add_library(steadywin_connect STATIC
    steadywin_connect/src/core/steadywin_motor.cpp
    steadywin_connect/src/core/motor_manager.cpp
    steadywin_connect/src/hal/gs_usb_can_port.cpp
    steadywin_connect/src/hal/serial_can_port.cpp
    steadywin_connect/src/hal/windows_serial_port.cpp
    steadywin_connect/src/protocol/steadywin_protocol_can.cpp
    steadywin_connect/src/protocol/steadywin_protocol_rs485.cpp
    steadywin_connect/src/utils/udp_control_server.cpp
)

# --- Include Directories ---
target_include_directories(steadywin_connect PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/steadywin_connect/include
    ${CMAKE_CURRENT_SOURCE_DIR}/CAN_CONNECT/third_party/libusb/include
)

# --- Platform-dependent libraries ---
if(WIN32)
    # Link against libusb (static) and system libraries
    # SetupAPI is needed for serial port enumeration and device discovery
    # Using MinGW64 static library
    target_link_libraries(steadywin_connect PRIVATE 
        setupapi 
        ws2_32
        ${CMAKE_CURRENT_SOURCE_DIR}/CAN_CONNECT/third_party/libusb/MinGW64/static/libusb-1.0.a
    )
    
    # Copy libusb-1.0.dll to build directory for runtime (from MinGW64/dll just in case static lib needs it or we switch to dll)
    # Note: Static lib usually doesn't need DLL, but if we use the DLL version we need it.
    # The 'static' folder usually contains .a which includes the code. 
    # However, let's copy the DLL anyway to be safe, or if we decide to link against dll.a later.
    add_custom_command(TARGET steadywin_connect POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/CAN_CONNECT/third_party/libusb/MinGW64/dll/libusb-1.0.dll
        $<TARGET_FILE_DIR:steadywin_connect>
    )
endif()


# --- Add example executable ---
add_executable(example_app src/main.cpp)

# Link example with our library
target_link_libraries(example_app PRIVATE steadywin_connect)

# --- Add telemetry/control app for 6 motors ---
add_executable(telemetry_6motors_app
    src/main_telemetry_6motors.cpp
    src/six_motor_controller.cpp
)

target_link_libraries(telemetry_6motors_app PRIVATE steadywin_connect)

# Ensure DLL is also copied to executable directory
if(WIN32)
    add_custom_command(TARGET example_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/CAN_CONNECT/third_party/libusb/MinGW64/dll/libusb-1.0.dll
        $<TARGET_FILE_DIR:example_app>
    )

    add_custom_command(TARGET telemetry_6motors_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/CAN_CONNECT/third_party/libusb/MinGW64/dll/libusb-1.0.dll
        $<TARGET_FILE_DIR:telemetry_6motors_app>
    )
endif()

# --- Tests ---
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    add_executable(can_debug_test tests/can_debug_test.cpp)
    target_link_libraries(can_debug_test PRIVATE steadywin_connect)

    add_executable(can_scan_robust_test tests/can_scan_robust_test.cpp)
    target_link_libraries(can_scan_robust_test PRIVATE steadywin_connect)

    add_executable(candlelight_test tests/candlelight_test.cpp)
    target_link_libraries(candlelight_test PRIVATE steadywin_connect)

    add_executable(winusb_bare_test tests/winusb_bare_test.cpp)
    target_link_libraries(winusb_bare_test PRIVATE steadywin_connect)
endif()

message(STATUS "Configuration complete. LibUSB integration enabled.")
