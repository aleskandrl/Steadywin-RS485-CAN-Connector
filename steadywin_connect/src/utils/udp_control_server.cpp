#include "steadywin/core/utils/udp_control_server.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

// Platform-specific socket includes
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace steadywin {
namespace utils {

UdpControlServer::UdpControlServer(steadywin::MotorManager& manager)
    : manager_(manager) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

UdpControlServer::~UdpControlServer() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpControlServer::start(const std::string& remote_host, uint16_t remote_port, uint16_t listen_port) {
    if (is_running_) {
        return true; // Already running
    }

    remote_host_ = remote_host;
    remote_port_ = remote_port;
    listen_port_ = listen_port;

    // --- Create listening socket (simplified) ---
    listen_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (listen_socket_fd_ < 0) {
        std::cerr << "UDP Server: Failed to create socket." << std::endl;
        return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(listen_port_);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "UDP Server: Failed to bind socket to port " << listen_port_ << std::endl;
        return false;
    }

    is_running_ = true;
    command_thread_ = std::thread(&UdpControlServer::commandLoop, this);
    telemetry_thread_ = std::thread(&UdpControlServer::telemetryLoop, this);

    std::cout << "UDP Server started. Listening on port " << listen_port_
              << ", sending telemetry to " << remote_host_ << ":" << remote_port_ << std::endl;
    return true;
}

void UdpControlServer::stop() {
    if (!is_running_) {
        return;
    }
    is_running_ = false;

    // Unblock the listening socket
    #ifdef _WIN32
        closesocket(listen_socket_fd_);
    #else
        close(listen_socket_fd_);
    #endif

    if (command_thread_.joinable()) {
        command_thread_.join();
    }
    if (telemetry_thread_.joinable()) {
        telemetry_thread_.join();
    }
    std::cout << "UDP Server stopped." << std::endl;
}

bool UdpControlServer::isRunning() const {
    return is_running_;
}

void UdpControlServer::telemetryLoop() {
    // Create socket for sending telemetry
    int telemetry_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (telemetry_socket < 0) {
        std::cerr << "Telemetry: Failed to create socket." << std::endl;
        return;
    }

    sockaddr_in remote_addr{};
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(remote_port_);
    inet_pton(AF_INET, remote_host_.c_str(), &remote_addr.sin_addr);

    while (is_running_) {
        std::stringstream ss;
        ss << "DATA;";
        
        auto found_ids = manager_.getFoundMotorIds();
        
        for (int i = 1; i <= 10; ++i) {
            auto motor = manager_.getMotor(i);
            if (motor) {
                Telemetry t;
                if (motor->getTelemetry(t) == MotorError::Ok) {
                    ss << i << "," << (int)t.raw_running_status << "," << t.multi_turn_angle_deg
                       << "," << t.velocity_rpm << "," << (int)t.temperature_celsius << "," << (int)t.raw_fault_code;
                } else {
                     ss << i << ",0,0,0,0,1"; // Indicate communication error
                }
            } else {
                ss << i << ",0,0,0,0,0"; // Motor not present
            }
            if (i < 10) ss << ";";
        }
        ss << "\n";

        std::string packet = ss.str();
        sendto(telemetry_socket, packet.c_str(), packet.length(), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 Hz
    }

    #ifdef _WIN32
        closesocket(telemetry_socket);
    #else
        close(telemetry_socket);
    #endif
}

void UdpControlServer::commandLoop() {
    char buffer[1024];
    while (is_running_) {
        int bytes_received = recvfrom(listen_socket_fd_, buffer, sizeof(buffer) - 1, 0, NULL, NULL);
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            parseAndExecuteCommand(std::string(buffer));
        } else {
            // Error or socket closed, exit loop
            if (is_running_) {
                 std::cerr << "UDP command loop recvfrom error." << std::endl;
            }
            break;
        }
    }
}

void UdpControlServer::parseAndExecuteCommand(const std::string& command_str) {
    std::cout << "Received command: " << command_str << std::endl;
    std::stringstream ss(command_str);
    std::string segment;
    std::vector<std::string> segments;
    while(std::getline(ss, segment, ';')) {
       segments.push_back(segment);
    }

    if (segments.empty()) return;

    try {
        std::string cmd_type = segments[0];
        uint8_t id = std::stoi(segments[1]);
        auto motor = manager_.getMotor(id);
        if (!motor) return;

        if (cmd_type == "SET" && segments.size() >= 3) {
            std::string param_type = segments[2];
            if (param_type == "POS" && segments.size() == 4) {
                double pos = std::stod(segments[3]);
                motor->moveTo(pos);
            } else if (param_type == "VEL" && segments.size() == 4) {
                double vel = std::stod(segments[3]);
                motor->setVelocity(vel);
            }
            // Add PID tuning here if needed
        } else if (cmd_type == "CMD" && segments.size() >= 3) {
            std::string action = segments[2];
            if (action == "CLEARFAULT") {
                motor->clearFaults();
            } else if (action == "BRAKE" && segments.size() == 4) {
                bool closed = std::stoi(segments[3]) != 0;
                motor->setBrake(closed);
            } else if (action == "DISABLE") {
                motor->disable();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing command '" << command_str << "': " << e.what() << std::endl;
    }
}

} // namespace utils
} // namespace steadywin
