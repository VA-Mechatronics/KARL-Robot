#include "StepCounter.h"

StepCounter::StepCounter(TIM_HandleTypeDef* countTimer, uint32_t stepsPerRev)
    : countTimer_(countTimer), stepsPerRev_(stepsPerRev) {}

void StepCounter::begin() {
    if (countTimer_ != nullptr) {
        HAL_TIM_Base_Start(countTimer_);
        last_cnt_ = countTimer_->Instance->CNT; // Startwert einfrieren
    }
}

void StepCounter::update(bool current_dir_pin_high, bool is_enabled) {
    if (countTimer_ == nullptr) return;

    // 1. Read Hardware-Register 
    uint32_t current_cnt = countTimer_->Instance->CNT;

    // 2. Difference(32-Bit Overflows save)
    uint32_t delta = current_cnt - last_cnt_;
    last_cnt_ = current_cnt;

    // 3. Determine the polarity using the DIR pin
    // If your rotation direction is reversed, just swap true and false here!
    if (is_enabled) {}
        if (current_dir_pin_high) {
            total_steps_ += delta;
        } else {
            total_steps_ -= delta;
    }
}

int32_t StepCounter::getPosition() const {
    // Steps to mdeg
    int64_t mdeg = (total_steps_ * 360000LL) / (stepsPerRev_ * microsteps_);
    return (int32_t)mdeg;
}

void StepCounter::setPosition(int32_t target_mdeg) {
    if (countTimer_ == nullptr) return;

    // Synchronize the last CNT register to prevent false steps during the next update
    last_cnt_ = countTimer_->Instance->CNT;

    // Calculate the corresponding steps backward
    total_steps_ = ((int64_t)target_mdeg * stepsPerRev_ * microsteps_) / 360000LL;
}

void StepCounter::setMicrosteps(uint32_t microsteps) {
    if (microsteps == 0) return;
    
    // Important: Before changing the resolution, we convert the existing
    // steps to the new microstep resolution so that the position doesn't jump!
    total_steps_ = (total_steps_ * microsteps) / microsteps_;
    microsteps_ = microsteps;
}
