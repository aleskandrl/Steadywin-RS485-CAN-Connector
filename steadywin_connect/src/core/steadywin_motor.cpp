#include "steadywin/core/steadywin_motor.h"
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>

namespace steadywin {

// Note: The constructor signature has changed.
SteadywinMotor::SteadywinMotor(uint8_t device_address, std::shared_ptr<SteadywinProtocol> protocol, std::recursive_mutex& bus_mutex)
    : device_address_(device_address),
      protocol_(protocol),
      bus_mutex_(bus_mutex)
{
}

void SteadywinMotor::drainBuffer() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    // Transport-specific drain is not exposed in SteadywinProtocol interface yet.
    // Keeping this method as a safe no-op hook for startup/shutdown handshakes.
}

MotorError SteadywinMotor::initialize() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    // To initialize, we just read some data. If it succeeds, the motor is there.
    RealtimeDataPayload payload;
    MotorError result = protocol_->readRealtimeData(device_address_, payload);
    if (result == MotorError::Ok) {
        is_initialized_ = true;
        // Sync smoothing target with current position to avoid jumps
        accumulated_target_deg_ = payload.multi_turn_angle * COUNTS_TO_DEG;
        target_initialized_ = true;
    }
    return result;
}

MotorError SteadywinMotor::disable() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    RealtimeDataPayload response;
    return protocol_->disableMotor(device_address_, response);
}

MotorError SteadywinMotor::holdPosition() {
    // Hold position is achieved by moving to the current position.
    // First, get the current position.
    Telemetry current_telemetry;
    MotorError err = getTelemetry(current_telemetry); // getTelemetry is already locked
    if (err != MotorError::Ok) {
        return err;
    }
    // Now command a move to that position.
    return moveTo(current_telemetry.multi_turn_angle_deg);
}

MotorError SteadywinMotor::moveTo(double angle_degrees) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);

    if (!target_initialized_) {
        accumulated_target_deg_ = angle_degrees;
        target_initialized_ = true;
    } else {
        // Always apply EMA for smoothness
        accumulated_target_deg_ = (smoothing_factor_ * angle_degrees) + 
                                  ((1.0 - smoothing_factor_) * accumulated_target_deg_);
    }

    // Use nearest-integer conversion to avoid systematic truncation bias.
    int32_t target_counts = static_cast<int32_t>(std::llround(accumulated_target_deg_ * DEG_TO_COUNTS));
    RealtimeDataPayload response;
    return protocol_->setAbsolutePositionControl(device_address_, target_counts, response);
}

MotorError SteadywinMotor::moveRelative(double delta_degrees) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    // Use nearest-integer conversion to avoid losing precision on small moves.
    int32_t relative_counts = static_cast<int32_t>(std::llround(delta_degrees * DEG_TO_COUNTS));
    RealtimeDataPayload response;
    return protocol_->setRelativePositionControl(device_address_, relative_counts, response);
}

MotorError SteadywinMotor::setVelocity(double velocity_rpm) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    int32_t target_velocity_x100 = static_cast<int32_t>(velocity_rpm / RPM_UNIT);
    uint32_t acceleration = 1000; // Default acceleration for now
    RealtimeDataPayload response;
    return protocol_->setVelocityControl(device_address_, target_velocity_x100, acceleration, response);
}

MotorError SteadywinMotor::clearFaults() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    uint8_t current_faults;
    return protocol_->clearFaults(device_address_, current_faults);
}

MotorError SteadywinMotor::setZero() {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    uint16_t mechanical_offset;
    return protocol_->setZeroPoint(device_address_, mechanical_offset);
}

MotorError SteadywinMotor::setBrake(bool closed) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    // Inverted logic: 0x00 = Closed (Engaged), 0x01 = Open (Released)
    uint8_t operation = closed ? 0x00 : 0x01;
    uint8_t status;
    return protocol_->setBrakeControl(device_address_, operation, status);
}

MotorError SteadywinMotor::moveToNoWait(double angle_degrees) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    int32_t counts = static_cast<int32_t>(angle_degrees / COUNTS_TO_DEG);
    return protocol_->setAbsolutePositionControlNoResponse(device_address_, counts);
}

void SteadywinMotor::setSmoothingFactor(double alpha) {
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    smoothing_factor_ = alpha;
}

MotorError SteadywinMotor::setPositionSpeedLimit(double rpm) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    MotionControlParametersPayload params;
    // First, read current params
    MotorError err = protocol_->readMotionControlParameters(device_address_, params);
    if (err != MotorError::Ok) {
        return err;
    }
    // Then, modify only the speed limit
    params.pos_limit_rpm_x100 = static_cast<uint32_t>(rpm / RPM_UNIT);
    // Finally, write them back
    return protocol_->writeMotionControlParameters(device_address_, params);
}

MotorError SteadywinMotor::setPIDs(float pos_kp, float pos_ki, float vel_kp, float vel_ki) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    MotionControlParametersPayload params;
    MotorError err = protocol_->readMotionControlParameters(device_address_, params);
    if (err != MotorError::Ok) return err;
    
    params.pos_kp = pos_kp;
    params.pos_ki = pos_ki;
    params.vel_kp = vel_kp;
    params.vel_ki = vel_ki;
    
    return protocol_->writeMotionControlParameters(device_address_, params);
}

MotorError SteadywinMotor::getPIDs(float& pos_kp, float& pos_ki, float& vel_kp, float& vel_ki) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    MotionControlParametersPayload params;
    MotorError err = protocol_->readMotionControlParameters(device_address_, params);
    if (err != MotorError::Ok) return err;
    
    pos_kp = params.pos_kp;
    pos_ki = params.pos_ki;
    vel_kp = params.vel_kp;
    vel_ki = params.vel_ki;
    return MotorError::Ok;
}

MotorError SteadywinMotor::moveToAndWait(double angle_degrees, unsigned int timeout_ms, double tolerance_deg) {
    auto start_time = std::chrono::steady_clock::now();

    MotorError err = moveTo(angle_degrees);
    if (err != MotorError::Ok) {
        return err;
    }

    while (true) {
        // Check for timeout
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed_ms > timeout_ms) {
            return MotorError::Timeout;
        }

        // Get current state
        Telemetry current_telemetry;
        err = getTelemetry(current_telemetry);
        if (err != MotorError::Ok) {
            return err; // Communication error
        }
        
        if (current_telemetry.raw_fault_code != 0) {
            return MotorError::DeviceReportedFault;
        }

        // Check if target is reached
        if (std::abs(current_telemetry.multi_turn_angle_deg - angle_degrees) <= tolerance_deg) {
            // Optional: wait for velocity to be near zero
            if (std::abs(current_telemetry.velocity_rpm) < 1.0) {
                 return MotorError::Ok;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

MotorError SteadywinMotor::moveToWithProfile(double angle_degrees, const VelocityControlProfile& profile, unsigned int timeout_ms) {
    // This is a conceptual implementation of a host-side trajectory generator.
    // NOTE: For true on-the-fly retargeting, `target_angle_deg` should be an atomic
    // or mutex-protected member variable. For this example, we treat it as constant.
    const double target_angle_deg = angle_degrees;

    auto start_time = std::chrono::steady_clock::now();
    const double control_loop_dt_s = 0.01; // 10ms control loop

    Telemetry current_telemetry;
    MotorError err = getTelemetry(current_telemetry);
    if (err != MotorError::Ok) return err;

    double current_velocity_rpm = current_telemetry.velocity_rpm;
    double acceleration_rpm_s = profile.acceleration_rpm_s;
    if (acceleration_rpm_s <= 0) acceleration_rpm_s = 500.0; // safety

    while (true) {
        // --- 1. Timing and Timeout Check ---
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
            setVelocity(0); // Stop the motor
            return MotorError::Timeout;
        }

        // --- 2. Get Feedback ---
        err = getTelemetry(current_telemetry);
        if (err != MotorError::Ok) return err;
        if (current_telemetry.raw_fault_code != 0) return MotorError::DeviceReportedFault;

        // --- 3. Check for Completion ---
        double remaining_dist_deg = target_angle_deg - current_telemetry.multi_turn_angle_deg;
        if (std::abs(remaining_dist_deg) <= profile.tolerance_deg) {
            setVelocity(0);
            return MotorError::Ok;
        }

        // --- 4. Trajectory Generation ---
        double direction = (remaining_dist_deg > 0) ? 1.0 : -1.0;
        
        // Calculate braking distance in degrees. V^2 = V0^2 + 2*a*d  => d = V^2 / (2*a)
        // Convert units: RPM to Deg/s, RPM/s to Deg/s^2
        double current_vel_dps = current_velocity_rpm * 360.0 / 60.0;
        double accel_dps2 = acceleration_rpm_s * 360.0 / 60.0;
        double braking_dist_deg = (current_vel_dps * current_vel_dps) / (2.0 * accel_dps2);

        double target_velocity_rpm = 0;

        // --- State Machine ---
        if (std::abs(remaining_dist_deg) <= braking_dist_deg) {
            // BRAKING PHASE: We are inside the braking window, must decelerate.
            // Simplified deceleration logic for this example.
            target_velocity_rpm = current_velocity_rpm - direction * acceleration_rpm_s * control_loop_dt_s;
        } else {
            // ACCELERATION / CRUISING PHASE
            target_velocity_rpm = current_velocity_rpm + direction * acceleration_rpm_s * control_loop_dt_s;
        }

        // --- 5. Saturation and Command ---
        // Clamp target velocity to profile limits
        target_velocity_rpm = std::min(target_velocity_rpm, profile.max_velocity_rpm);
        target_velocity_rpm = std::max(target_velocity_rpm, -profile.max_velocity_rpm);

        // Send the command
        setVelocity(target_velocity_rpm);
        current_velocity_rpm = target_velocity_rpm; // Assume motor follows command for next cycle calculation

        // --- 6. Loop Delay ---
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(control_loop_dt_s * 1000)));
    }
    
    return MotorError::UnknownError; // Should not be reached
}


MotorError SteadywinMotor::getTelemetry(Telemetry& telemetry) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    RealtimeDataPayload payload;
    MotorError result = protocol_->readRealtimeData(device_address_, payload);
    if (result == MotorError::Ok) {
        telemetry = convertPayloadToTelemetry(payload);
        last_telemetry_ = telemetry;

        // If not initialized yet, sync the smoothing target
        if (!target_initialized_) {
            accumulated_target_deg_ = telemetry.multi_turn_angle_deg;
            target_initialized_ = true;
        }
    }
    return result;
}

MotorError SteadywinMotor::getPositionFeedback(double& angle_deg) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    int32_t raw_counts;
    MotorError result = protocol_->readMultiTurnAngle(device_address_, raw_counts);
    if (result == MotorError::Ok) {
        angle_deg = raw_counts * COUNTS_TO_DEG;
    }
    return result;
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
    telemetry.single_turn_angle_deg = payload.single_turn_angle * COUNTS_TO_DEG;
    telemetry.multi_turn_angle_deg = payload.multi_turn_angle * COUNTS_TO_DEG;
    telemetry.velocity_rpm = payload.mechanical_velocity * RPM_UNIT;
    telemetry.q_axis_current_amps = payload.q_axis_current * CURRENT_UNIT;
    telemetry.bus_voltage_volts = payload.bus_voltage * VOLTAGE_UNIT;
    telemetry.bus_current_amps = payload.bus_current * 0.01; // Assuming same unit as voltage
    telemetry.temperature_celsius = payload.working_temperature;
    telemetry.raw_running_status = payload.running_status;
    telemetry.raw_motor_status = payload.motor_status;
    telemetry.raw_fault_code = payload.fault_code;
    return telemetry;
}

MotorError SteadywinMotor::setMitLimits(float p_max, float v_max, float t_max) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    auto can_proto = std::dynamic_pointer_cast<SteadywinProtocolCAN>(protocol_);
    if (!can_proto) return MotorError::UnknownError; // Not a CAN protocol or doesn't support MIT

    mit_limits_.p_max = p_max;
    mit_limits_.v_max = v_max;
    mit_limits_.t_max = t_max;

    return can_proto->configureMitLimits(device_address_, mit_limits_);
}

MotorError SteadywinMotor::sendMitControl(float p_des, float v_des, float kp, float kd, float t_ff, Telemetry& telemetry) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    auto can_proto = std::dynamic_pointer_cast<SteadywinProtocolCAN>(protocol_);
    if (!can_proto) return MotorError::UnknownError;

    RealtimeDataPayload payload;
    MotorError err = can_proto->sendMitCommand(device_address_, p_des, v_des, kp, kd, t_ff, mit_limits_, payload);
    
    if (err == MotorError::Ok) {
        telemetry = convertPayloadToTelemetry(payload);
        last_telemetry_ = telemetry;
        // Sync smoothing target? constant MIT control usually doesn't use the smoothing logic of moveTo.
        // But if we switch back to moveTo, we might want to sync.
        accumulated_target_deg_ = telemetry.multi_turn_angle_deg; 
        target_initialized_ = true;
    }
    return err;
}

void SteadywinMotor::setControlMode(ControlMode mode) {
    std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
    if (mode == ControlMode::Disabled) {
        requested_mode_ = ControlMode::RequestDisable;
    } else {
        // If we are currently disabled, we need to re-enable/initialize targets
        if (control_mode_ == ControlMode::Disabled || control_mode_ == ControlMode::RequestDisable) {
             // Capture current position as target to avoid jumps on enable
             Telemetry t;
             if (getTelemetry(t) == MotorError::Ok) {
                 target_angle_deg_ = t.multi_turn_angle_deg;
                 current_cmd_angle_ = target_angle_deg_;
                 accumulated_target_deg_ = target_angle_deg_;
                 sine_params_.center_deg = target_angle_deg_;
                 sine_time_accumulator_ = 0;
             }
        }
        control_mode_ = mode;
        requested_mode_ = mode;
    }
}

void SteadywinMotor::update(double dt_seconds) {
    if (!is_initialized_) return;

    // 1. Protection Check (Telemetery must be read externally or here? 
    //    Ideally we read telemetry here but that might be too slow if done synchronously for 6 motors.
    //    For now, we rely on `last_telemetry_` being updated by `getTelemetry` calls from the manager/main loop 
    //    OR we do a quick check here if needed. 
    //    The original plan implied main loop reads telemetry. Let's assume `last_telemetry_` is up to date enough 
    //    or we perform a check if `dt` indicates we should.)
    
    // Check Voltage Protection
    if (control_mode_ != ControlMode::Disabled && control_mode_ != ControlMode::RequestDisable) {
        if (last_telemetry_.bus_voltage_volts < min_voltage_limit_ && last_telemetry_.bus_voltage_volts > 1.0) {
            std::cerr << "Motor " << (int)device_address_ << " Under-Voltage Protection! (" << last_telemetry_.bus_voltage_volts << "V)" << std::endl;
            setControlMode(ControlMode::Disabled);
        }
        if (last_telemetry_.temperature_celsius > max_temp_limit_) {
             std::cerr << "Motor " << (int)device_address_ << " Over-Temp Protection! (" << last_telemetry_.temperature_celsius << "C)" << std::endl;
             setControlMode(ControlMode::Disabled);
        }
    }

    // 2. State Machine
    if (requested_mode_ == ControlMode::RequestDisable) {
        disable();
        control_mode_ = ControlMode::Disabled;
        requested_mode_ = ControlMode::Disabled;
        return;
    }

    if (control_mode_ == ControlMode::Disabled) {
        return; // Do nothing
    }

    // 3. Command Generation
    double final_command_deg = target_angle_deg_;

    if (control_mode_ == ControlMode::Sinusoid) {
        sine_time_accumulator_ += dt_seconds;
        constexpr double TWO_PI = 6.28318530717958647692;
        double sine_val = std::sin(TWO_PI * sine_params_.frequency_hz * sine_time_accumulator_ + sine_params_.phase_offset_rad);
        final_command_deg = sine_params_.center_deg + sine_params_.amplitude_deg * sine_val;
    }

    // 4. Interpolation / Rate Limiting (Simple Slew Rate for now)
    //    In standard Position mode, limits are handled by the motor's internal profile usually, 
    //    but for smooth host-side control we interpolate.
    if (control_mode_ == ControlMode::Position || control_mode_ == ControlMode::Sinusoid || control_mode_ == ControlMode::Hold) {
        // Apply smoothing factor (EMA) or exact following
        // Reuse smoothing_factor_ from existing class logic
        current_cmd_angle_ = (smoothing_factor_ * final_command_deg) + ((1.0 - smoothing_factor_) * current_cmd_angle_);
        
        // TODO: Apply Velocity Limits (pos_speed_limit_rpm_) here if needed 
        // For now, we trust the smoothing factor and motor's internal limits
        
        moveToNoWait(current_cmd_angle_);
    }
    else if (control_mode_ == ControlMode::Velocity) {
        setVelocity(target_velocity_rpm_);
    }
    else if (control_mode_ == ControlMode::Mit) {
         // MIT Mode params should be set via specific setters or struct. 
         // For now assumes limits and gains are set.
         // This requires expanding the update logic to support MIT params streaming if they change dynamically.
         // Current implementation in main.cpp streams them. 
         // We might need a `setMitTarget(...)` method. 
         // For now, if in MIT mode, we might assume standard update isn't enough OR 
         // we use `sendMitControl` with stored params.
         // Let's defer MIT active loop to `sendMitControl` called by user, OR 
         // implement `mit_target_` struct. 
         // Given the complexity, let's keep MIT manual for now or add a basic placeholder.
    }
}

} // namespace steadywin
