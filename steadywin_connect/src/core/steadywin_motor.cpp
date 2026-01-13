#include "steadywin/steadywin_motor.h"
#include <chrono>
#include <thread>
#include <cmath> // For std::abs

namespace steadywin {

SteadywinMotor::SteadywinMotor(uint8_t device_address, std::unique_ptr<SteadywinProtocol> protocol)
    : device_address_(device_address), protocol_(std::move(protocol)) {
}

MotorError SteadywinMotor::initialize() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    // To initialize, we just try to get the first telemetry packet.
    // If it succeeds, we know the motor is connected and responding.
    MotorError result = getTelemetry(last_telemetry_);
    if (result == MotorError::Ok) {
        is_initialized_ = true;
    }
    return result;
}

MotorError SteadywinMotor::disable() {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    RealtimeDataPayload response_payload;
    return protocol_->disableMotor(device_address_, response_payload);
}

MotorError SteadywinMotor::holdPosition()
{
    if (!is_initialized_) return MotorError::NotInitialized;

    // First, get the current position
    MotorError result = getTelemetry(last_telemetry_);
    if (result != MotorError::Ok) {
        return result;
    }
    
    // Then, command the motor to move to that exact position
    return moveTo(last_telemetry_.multi_turn_angle_deg);
}

MotorError SteadywinMotor::moveTo(double angle_degrees) {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);

    // Convert degrees to encoder counts
    const int32_t target_counts = static_cast<int32_t>(angle_degrees * DEG_TO_COUNTS);

    RealtimeDataPayload response_payload;
    return protocol_->setAbsolutePositionControl(device_address_, target_counts, response_payload);
}

MotorError SteadywinMotor::moveRelative(double delta_degrees) {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    const int32_t delta_counts = static_cast<int32_t>(delta_degrees * DEG_TO_COUNTS);
    RealtimeDataPayload response_payload;
    return protocol_->setRelativePositionControl(device_address_, delta_counts, response_payload);
}

MotorError SteadywinMotor::setVelocity(double velocity_rpm) {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    const int32_t velocity_x100 = static_cast<int32_t>(velocity_rpm / RPM_UNIT);
    RealtimeDataPayload response_payload;
    return protocol_->setVelocityControl(device_address_, velocity_x100, 0, response_payload); // 0 for max accel
}

MotorError SteadywinMotor::clearFaults() {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    uint8_t current_faults;
    return protocol_->clearFaults(device_address_, current_faults);
}

MotorError SteadywinMotor::setZero() {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    uint16_t mechanical_offset;
    return protocol_->setZeroPoint(device_address_, mechanical_offset);
}

MotorError SteadywinMotor::setBrake(bool closed) {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    uint8_t status;
    return protocol_->setBrakeControl(device_address_, closed ? 0x01 : 0x00, status);
}

MotorError SteadywinMotor::setPositionSpeedLimit(double rpm) {
    if (!is_initialized_) return MotorError::NotInitialized;
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    MotionControlParametersPayload params;
    // First read current to keep other values (KP/KI)
    MotorError result = protocol_->readMotionControlParameters(device_address_, params);
    if (result != MotorError::Ok) return result;
    
    params.pos_limit_rpm_x100 = static_cast<uint32_t>(rpm / 0.01);
    return protocol_->writeMotionControlParameters(device_address_, params);
}

MotorError SteadywinMotor::moveToAndWait(double angle_degrees, unsigned int timeout_ms, double tolerance_deg) {
    // Start the move
    MotorError result = moveTo(angle_degrees);
    if (result != MotorError::Ok) {
        return result;
    }

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Check for timeout
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed_ms >= timeout_ms) {
            return MotorError::Timeout;
        }

        // Get current telemetry
        Telemetry current_telemetry;
        result = getTelemetry(current_telemetry);
        if (result != MotorError::Ok) {
            // If communication fails, we can't continue
            return result;
        }

        // Check for device faults
        if (current_telemetry.raw_fault_code != 0) {
            return MotorError::DeviceReportedFault;
        }

        // Check if target is reached
        if (std::abs(current_telemetry.multi_turn_angle_deg - angle_degrees) <= tolerance_deg) {
            return MotorError::Ok;
        }

        // Wait a bit before the next poll to avoid spamming the bus
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

MotorError SteadywinMotor::getTelemetry(Telemetry& telemetry) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    RealtimeDataPayload payload;
    MotorError result = protocol_->readRealtimeData(device_address_, payload);

    if (result == MotorError::Ok) {
        telemetry = convertPayloadToTelemetry(payload);
    }
    return result;
}

MotorError SteadywinMotor::moveToWithProfile(double angle_degrees, const VelocityControlProfile& profile, unsigned int timeout_ms) {
    if (!is_initialized_) return MotorError::NotInitialized;

    auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;
    double integral_error = 0.0;
    double last_error = 0.0;
    double current_commanded_velocity = 0.0;
    bool first_loop = true;

    // Get initial velocity to start ramping from it
    Telemetry initial_telemetry;
    if (getTelemetry(initial_telemetry) == MotorError::Ok) {
        current_commanded_velocity = initial_telemetry.velocity_rpm;
    }

    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed_total_ms >= timeout_ms) {
            setVelocity(0.0); // Stop the motor
            return MotorError::Timeout;
        }

        double dt = std::chrono::duration<double>(now - last_time).count();
        if (dt <= 0.0) dt = 0.001; // Prevent division by zero
        last_time = now;

        Telemetry telemetry;
        MotorError res = getTelemetry(telemetry);
        if (res != MotorError::Ok) return res;

        if (telemetry.raw_fault_code != 0) {
            setVelocity(0.0);
            return MotorError::DeviceReportedFault;
        }

        double current_angle = telemetry.multi_turn_angle_deg;
        double error = angle_degrees - current_angle;

        if (std::abs(error) <= profile.tolerance_deg && std::abs(telemetry.velocity_rpm) < 1.0) {
            setVelocity(0.0);
            return MotorError::Ok;
        }

        // PID Control
        integral_error += error * dt;
        
        // Anti-windup: clamp integral term
        double max_i_term = profile.max_velocity_rpm * 0.5; 
        if (profile.i_gain > 0) {
            if (integral_error * profile.i_gain > max_i_term) integral_error = max_i_term / profile.i_gain;
            if (integral_error * profile.i_gain < -max_i_term) integral_error = -max_i_term / profile.i_gain;
        }

        double derivative_error = first_loop ? 0.0 : (error - last_error) / dt;
        
        double requested_velocity = (profile.p_gain * error) + 
                                   (profile.i_gain * integral_error) + 
                                   (profile.d_gain * derivative_error);

        // Clamp requested velocity to max limit
        if (requested_velocity > profile.max_velocity_rpm) requested_velocity = profile.max_velocity_rpm;
        if (requested_velocity < -profile.max_velocity_rpm) requested_velocity = -profile.max_velocity_rpm;

        // Apply Acceleration Limiting (Ramping)
        double max_dv = profile.acceleration_rpm_s * dt;
        double velocity_diff = requested_velocity - current_commanded_velocity;
        
        if (std::abs(velocity_diff) > max_dv) {
            if (velocity_diff > 0) current_commanded_velocity += max_dv;
            else current_commanded_velocity -= max_dv;
        } else {
            current_commanded_velocity = requested_velocity;
        }

        res = setVelocity(current_commanded_velocity);
        if (res != MotorError::Ok) return res;

        last_error = error;
        first_loop = false;

        // Loop frequency control - ~50Hz is reasonable for host-side control over RS485
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool SteadywinMotor::hasFault() {
    Telemetry temp;
    if (getTelemetry(temp) == MotorError::Ok) {
        return temp.raw_fault_code != 0;
    }
    return true; // Assume fault on communication error
}

Telemetry SteadywinMotor::convertPayloadToTelemetry(const RealtimeDataPayload& payload) {
    Telemetry telemetry;
    telemetry.single_turn_angle_deg = static_cast<double>(payload.single_turn_angle) * COUNTS_TO_DEG;
    telemetry.multi_turn_angle_deg = static_cast<double>(payload.multi_turn_angle) * COUNTS_TO_DEG;
    telemetry.velocity_rpm = static_cast<double>(payload.mechanical_velocity) * RPM_UNIT;
    telemetry.q_axis_current_amps = static_cast<double>(payload.q_axis_current) * CURRENT_UNIT;
    telemetry.bus_voltage_volts = static_cast<double>(payload.bus_voltage) * VOLTAGE_UNIT;
    telemetry.bus_current_amps = static_cast<double>(payload.bus_current) * CURRENT_UNIT;
    telemetry.temperature_celsius = static_cast<int8_t>(payload.working_temperature);
    telemetry.raw_running_status = payload.running_status;
    telemetry.raw_motor_status = payload.motor_status;
    telemetry.raw_fault_code = payload.fault_code;
    return telemetry;
}

} // namespace steadywin
