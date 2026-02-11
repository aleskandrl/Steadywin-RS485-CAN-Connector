#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "steadywin/core/motor_manager.h"

namespace steadywin_app {

class SixMotorController {
public:
    struct MotorSnapshot {
        uint8_t id{0};
        bool telemetry_ok{false};
        bool enabled{false};
        bool brake_closed{true};
        double angle_deg{0.0};
        double velocity_rpm{0.0};
        double current_a{0.0};
        double voltage_v{0.0};
        double exchange_hz{0.0};
        uint8_t raw_fault_code{0};
        std::string alarm;
    };

    struct Snapshot {
        std::vector<MotorSnapshot> motors;
        size_t active_index{0};
        std::vector<std::string> logs;
    };

    struct MotorSlot {
        uint8_t id{0};
        std::shared_ptr<steadywin::SteadywinMotor> motor;
        steadywin::Telemetry telemetry{};
        bool telemetry_ok{false};

        bool enabled{false};
        bool brake_closed{true};
        bool limit_initialized{false};
        double initial_angle_on_enable_deg{0.0};
        double target_angle_deg{0.0};

        double exchange_hz{0.0};
        std::chrono::steady_clock::time_point last_ok_telemetry{};
        bool has_last_ok_telemetry{false};

        std::string alarm;
    };

    explicit SixMotorController(std::unique_ptr<steadywin::MotorManager> manager);
    ~SixMotorController();

    bool scanAndAttachMotors(uint8_t max_address_to_scan = 10, size_t max_motors = 6);
    void setActiveIndex(size_t index);
    void stepActiveSelection(int delta);

    bool enableActiveMotor();
    bool disableActiveMotor();
    bool toggleBrakeActiveMotor();
    bool stepActiveMotor(double delta_deg);

    void startPolling();
    void stopPolling();

    Snapshot snapshot() const;
    size_t motorCount() const;
    void disableAll();
    
    // Accessors for external control loop
    std::shared_ptr<steadywin::SteadywinMotor> getMotorByIndex(size_t index);
    bool isMotorEnabled(size_t index) const;
    void updateMotorTelemetry(size_t index, const steadywin::Telemetry& t);
    void updateMotorAngle(size_t index, double angle_deg);
    void processTelemetryAndProtection(size_t index, const steadywin::Telemetry& t);
    const std::vector<MotorSlot>& motors() const { return motors_; }

private:


    static constexpr double kAngleLimitDeg = 40.0;
    static constexpr double kMinBusVoltageV = 30.0;
    static constexpr double kMaxCurrentA = 3.0;

    std::unique_ptr<steadywin::MotorManager> manager_;
    mutable std::mutex mutex_;

    std::vector<MotorSlot> motors_;
    size_t active_index_{0};
    std::deque<std::string> logs_;

    std::thread polling_thread_;
    bool polling_running_{false};

    MotorSlot* activeSlotLocked() {
        if (motors_.empty()) return nullptr;
        if (active_index_ >= motors_.size()) active_index_ = 0;
        return &motors_[active_index_];
    }

    void addLogLocked(const std::string& message);
    void stopPollingLocked(std::thread& worker);
    void triggerProtectionLocked(MotorSlot& slot, const std::string& reason);
    void pollingLoop();

    static std::string toFixed(double value, int precision);
};

} // namespace steadywin_app
