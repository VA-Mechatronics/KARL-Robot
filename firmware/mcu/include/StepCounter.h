#pragma once
#include "stm32g4xx_hal.h"
#include <stdint.h>

class StepCounter {
public:
    StepCounter(TIM_HandleTypeDef* countTimer, uint32_t stepsPerRev = 200);

    void begin();

    void update(bool current_dir_pin_high, bool is_enabled);

    int32_t getPosition() const;

    void setPosition(int32_t target_mdeg);
    void setMicrosteps(uint32_t microsteps);

private:
    TIM_HandleTypeDef* countTimer_;
    
    uint32_t stepsPerRev_;
    uint32_t microsteps_ = 16; // Standardwert passend zum TMC-Initialwert

    int64_t total_steps_ = 0;
    uint32_t last_cnt_ = 0;
};
