#pragma once
#include <stdint.h>
#include "Encoder.h"

extern Encoder encoder;

extern "C" uint32_t HAL_GetTick(void);

struct TrajectoryPoint {
    int32_t pos;
    int32_t vel;
    uint32_t mcu_timestamp;
};

enum class ControllerMode {
    TRAJECTORY,
    GUIDING,
};

class PositionController {
public:
    // Kp, Ki, Kd, and Kv are passed as scaled integers (fixed-point * 1000)
    // Example: For Kp = 2.5, pass kp_scaled = 2500
    PositionController(
        int32_t kp_scaled, int32_t ki_scaled, int32_t kd_scaled, int32_t kv_scaled, int32_t gr_scaled,
        int32_t mode, int32_t guiding_gain, int32_t guiding_deadzone, int32_t error_limit
    )
        : kp_(kp_scaled), ki_(ki_scaled), kd_(kd_scaled), kv_(kv_scaled), gr_(gr_scaled),
        mode_(mode), guiding_gain_(guiding_gain), guiding_deadzone_(guiding_deadzone), error_limit_(error_limit) {}

    // Called in the MAIN LOOP (therefore protected against interrupts)
    bool addTrajectoryPoint(int32_t target_pos, int32_t target_vel, uint16_t pi_timestamp) {
        uint8_t next_write = (write_idx_ + 1) % QUEUE_SIZE;
        if (next_write == read_idx_) {
            // queue full, RPi to fast
            return false;
        }

        uint32_t current_mcu_time = HAL_GetTick();
        uint32_t target_mcu_time = 0;

        if (queue_empty() && !is_streaming_) {
            // FIRST PACKET / RESTART: Establish relative synchronization
            // We assume the first packet is valid in 20 ms (2 control cycles)
            target_mcu_time = current_mcu_time + 20;
            pi_start_timestamp_ = pi_timestamp;
            mcu_start_timestamp_ = target_mcu_time;
            is_streaming_ = true;
        } else {
            // FOLLOW-UP PACKAGES: Calculate timestamps relative only to the starting point
            // Prevents problems with 16-bit wrap-around of the Pi timestamp
            uint16_t pi_delta = (pi_timestamp - pi_start_timestamp_) & 0xFFFF;
            target_mcu_time = mcu_start_timestamp_ + pi_delta;
            // Validity check: If the time is in the past, reject it immediately
            if (target_mcu_time < current_mcu_time && (current_mcu_time - target_mcu_time) < 30000) {
                return true;
            }
        }
        queue_[write_idx_].pos = target_pos;
        queue_[write_idx_].vel = target_vel;
        queue_[write_idx_].mcu_timestamp = target_mcu_time;
        write_idx_ = next_write;
        return true;
    }

    void clearQueue(void) {
        read_idx_ = 0;
        write_idx_ = 0;
        is_streaming_ = false; // Forces a new relative time synchronization on the next startup
        last_error_ = 0;
        i_term_accumulator = 0;
        
        // Set the “last active point” to the current actual position,
        // so that the controller stops exactly here when at rest.
        last_active_point_.pos = encoder.getPosition(); 
        last_active_point_.vel = 0;
        last_active_point_.mcu_timestamp = HAL_GetTick();
    }

    // Called in the 10 ms timer interrupt (TIM3)
    // Returns the target speed in mdeg/s for your TMC5160::setVelocity
    int64_t update(int32_t actual_pos) {
        switch (mode_) {
            case 0:
                return update_controlling(actual_pos);
                break;
            case 1:
                return update_guiding(actual_pos);
                break;
            default:
                return 0;
                break;
        }
    }

    // Tuning interfaces for your configuration system (Expects scaled integers * 1000)
    void setKp(int32_t kp) { kp_ = kp; }
    void setKi(int32_t ki) { ki_ = ki; i_term_accumulator = 0; }
    void setKd(int32_t kd) { kd_ = kd; }
    void setKv(int32_t kv) { kv_ = kv; }
    void setGr(int32_t gr) { gr_ = gr; }
    int32_t getKp(void) const { return kp_; }
    int32_t getKi(void) const { return ki_; }
    int32_t getKd(void) const { return kd_; }
    int32_t getKv(void) const { return kv_; }
    int32_t getGr(void) const { return gr_; }
    int32_t getError(void) const { return last_error_; }
    int32_t getOutput(void) const {return velocity_output; }

    void setMode(int32_t mode) {
        if (mode_ != mode) {
            mode_ = mode;
            clearQueue();
        }
    }
    int32_t getMode() const { return mode_; }
    void setGuidingGain(int32_t gain) { guiding_gain_ = gain; }
    int32_t getGuidingGain() const { return guiding_gain_; }
    void setGuidingDeadzone(int32_t deadzone) { guiding_deadzone_ = deadzone; }
    int32_t getGuidingDeadzone() const { return guiding_deadzone_; }
    void setErrorLimit(int32_t error_limit) { error_limit_ = error_limit; }
    int32_t getErrorLimit() const { return error_limit_; }

    uint8_t getQueueFill() const {
        if (write_idx_ >= read_idx_) return write_idx_ - read_idx_;
        return (QUEUE_SIZE - read_idx_) + write_idx_;
    }
    

private:
    static constexpr uint8_t QUEUE_SIZE = 32;
    TrajectoryPoint queue_[QUEUE_SIZE];
    volatile uint8_t read_idx_ = 0;
    volatile uint8_t write_idx_ = 0;

    bool is_streaming_ = false;
    uint16_t pi_start_timestamp_ = 0;
    uint32_t mcu_start_timestamp_ = 0;
    
    TrajectoryPoint last_active_point_ = {0, 0, 0};

    int32_t kp_, ki_, kd_, kv_, gr_;
    int64_t i_term_accumulator = 0;
    int64_t last_error_ = 0;
    int64_t velocity_output = 0;

    int32_t mode_;
    int32_t guiding_gain_;
    int32_t guiding_deadzone_;
    int32_t error_limit_;

    int64_t update_controlling(int32_t actual_pos) {
                uint32_t now = HAL_GetTick();

        // 1. Clear the queue and find the current interpolation endpoint
        advance_queue(now);

        // 2. Calculate setpoints for EXACTLY THIS 10-millisecond step
        int32_t current_set_pos = 0;
        int32_t current_set_vel = 0;
        interpolate(now, current_set_pos, current_set_vel);

        // --- PID ---
        int64_t error = current_set_pos - actual_pos; // mdeg
        int64_t p_term = ((int64_t)kp_ * error) / 1000LL;
        
        // dt = 10 ms = 0.01 s = 1/1000 s:
        // i_term += (ki_scaled * error) / (1000 * 1000)
        i_term_accumulator += ((int64_t)ki_ * error) / 1000000LL;
        // Anti-Windup
        if (i_term_accumulator > 90000LL)  i_term_accumulator = 90000LL;
        if (i_term_accumulator < -90000LL) i_term_accumulator = -90000LL;

        // dt = 0.01 s = 1/100 s:
        // d_term = (kd_scaled * d_input * 1000) / 1000
        int64_t d_input = error - last_error_;
        int64_t d_term = ((int64_t)kd_ * d_input * 1000LL) / 1000LL; 
        last_error_ = error;

        // Feed-Forward in mdeg/s
        int64_t ff_term = ((int64_t)kv_ * (int64_t)current_set_vel) / 1000LL;
        
        // PID sum
        velocity_output = (p_term + i_term_accumulator + d_term + ff_term); // mdeg/s

        // Gear correction
        int64_t motor_output = (velocity_output * (int64_t)gr_) / 1000LL;

        if (abs(error) > error_limit_) {
            clearQueue();
            motor_output = 0;
        }

        return motor_output; // mdeg/s
    }

    int64_t update_guiding(int32_t actual_pos) {
        // The “setpoint” is the position where we want to stop.
        // When the button is pressed, actual_pos is either lagging behind or running ahead.
        int32_t error = actual_pos - last_active_point_.pos; 

        // Deadzone
        if (error > guiding_deadzone_) {
            error -= guiding_deadzone_;
        } else if (error < -guiding_deadzone_) {
            error += guiding_deadzone_;
        } else {
            error = 0;
        }

        // Calculate speed proportional to the error (admittance approach)
        // guiding_gain_ scales how fast the motor runs (e.g., 500 = factor 0.5)
        velocity_output = (error * guiding_gain_) / 1000;

        // Limiter
        if (velocity_output > 40000) velocity_output = 40000;
        if (velocity_output < -40000) velocity_output = -40000;

        // In guiding mode, we continuously adjust the “setpoint” to match the actual position,
        // based on the speed we command the motor to maintain.
        // dt = 10 ms -> Position changes by (speed * 0.01 s)
        int32_t pos_delta = velocity_output / 1000; 
        last_active_point_.pos += pos_delta;
        last_active_point_.mcu_timestamp = HAL_GetTick();
        last_active_point_.vel = velocity_output;

        last_error_ = error; // Diagnose

        // Gear correction
        int32_t motor_output = (velocity_output * gr_) / 1000;
        return motor_output;
    }

    bool queue_empty() const { return read_idx_ == write_idx_; }

    void advance_queue(uint32_t now) {
        while (!queue_empty()) {
            if (queue_[read_idx_].mcu_timestamp > now) {
                break; // This point is in the future; cancel
            }
            // The point is in the past -> becomes the new starting point
            last_active_point_ = queue_[read_idx_];
            read_idx_ = (read_idx_ + 1) % QUEUE_SIZE;
        }
    }

    void interpolate(uint32_t now, int32_t& out_pos, int32_t& out_vel) {
        if (queue_empty()) {
            // GOAL ACHIEVED: Pi is no longer transmitting!
            // Maintaining the position of the last known point.
            out_pos = last_active_point_.pos;
            out_vel = 0; // Disable FF; PID compensates for the remaining tracking error
            is_streaming_ = false; // Sync reset
            return;
        }

        const auto& next_point = queue_[read_idx_];
        uint32_t t_start = last_active_point_.mcu_timestamp;
        uint32_t t_end = next_point.mcu_timestamp;

        if (now <= t_start || t_end <= t_start) {
            out_pos = next_point.pos;
            out_vel = next_point.vel;
            return;
        }

        // Linear factor (0 to 1000) between the two points
        int64_t factor = ((int64_t)(now - t_start) * 1000) / (t_end - t_start);

        out_pos = last_active_point_.pos + (int32_t)(((int64_t)(next_point.pos - last_active_point_.pos) * factor) / 1000);
        out_vel = last_active_point_.vel + (int32_t)(((int64_t)(next_point.vel - last_active_point_.vel) * factor) / 1000);
    }
};
