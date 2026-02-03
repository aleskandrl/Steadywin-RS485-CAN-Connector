#pragma once

#include "steadywin/core/motor_manager.h"
#include <string>
#include <cstdint>
#include <thread>
#include <atomic>
#include <memory>

namespace steadywin {
namespace utils {

/**
 * @class UdpControlServer
 * @brief An optional utility for remote motor control and telemetry via UDP.
 * This class provides a bridge for applications like MATLAB or Python to
 * interact with the motors managed by a MotorManager instance. It runs
 * networking operations in separate background threads.
 */
class UdpControlServer {
public:
    /**
     * @brief Constructor.
     * @param manager A reference to an initialized MotorManager.
     */
    explicit UdpControlServer(steadywin::MotorManager& manager);

    /**
     * @brief Destructor. Automatically stops the server threads.
     */
    ~UdpControlServer();

    // Disable copy and move semantics
    UdpControlServer(const UdpControlServer&) = delete;
    UdpControlServer& operator=(const UdpControlServer&) = delete;

    /**
     * @brief Starts the UDP server and telemetry streaming.
     * Spawns background threads for sending telemetry and listening for commands.
     * @param remote_host The IP address of the remote client (e.g., MATLAB PC).
     * @param remote_port The UDP port on the remote client to send telemetry to.
     * @param listen_port The local UDP port to listen on for incoming commands.
     * @return True if the server started successfully, false otherwise.
     */
    bool start(const std::string& remote_host, uint16_t remote_port, uint16_t listen_port);

    /**
     * @brief Stops the server threads gracefully.
     */
    void stop();

    /**
     * @brief Checks if the server is currently running.
     * @return True if the threads are active, false otherwise.
     */
    bool isRunning() const;

private:
    /**
     * @brief The main loop for the telemetry thread.
     * Periodically queries all motors for data and sends it in a single UDP packet.
     */
    void telemetryLoop();

    /**
     * @brief The main loop for the command listener thread.
     * Blocks on receiving a UDP packet and processes the command.
     */
    void commandLoop();

    /**
     * @brief Parses a command string received via UDP and executes it.
     * @param command_str The raw string from the UDP packet.
     */
    void parseAndExecuteCommand(const std::string& command_str);

    steadywin::MotorManager& manager_;
    
    // --- Threading and State ---
    std::thread telemetry_thread_;
    std::thread command_thread_;
    std::atomic<bool> is_running_{false};

    // --- Network Configuration ---
    // Note: Implementation details (e.g., socket handles) are OS-dependent and
    // are omitted from the header. A real implementation would use a library
    // like Boost.Asio or native sockets (e.g. Winsock, Berkeley sockets).
    std::string remote_host_;
    uint16_t remote_port_{0};
    uint16_t listen_port_{0};
    // Placeholder for socket descriptor
    int listen_socket_fd_{-1};
};

} // namespace utils
} // namespace steadywin