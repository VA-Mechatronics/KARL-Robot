#pragma once

#include <stdint.h>
#include "gpio.h"

struct DriverStatus
{
    bool ok = false;
    bool spiError = false;

    uint32_t drvStatus = 0;
    uint32_t gstat = 0;
    uint32_t ioin = 0;
    uint32_t pwmScale = 0;
    uint32_t pwmAuto  = 0;

    // decoded flag
    bool overTemp = false;
    bool overTempPrewarn = false;
    bool shortToGND = false;
    bool undervoltage = false;
    bool openLoad = false;
    bool standstill = false;

    uint16_t currentScale = 0;
    uint16_t sgResult = 0;
};

class StepperDriver {
public:
    virtual ~StepperDriver() = default;

    virtual void begin() = 0;
    virtual DriverStatus getStatus() = 0;

    virtual void enable(bool en) = 0;
    virtual void setVelocity(int64_t vel_mdeg_s) = 0;

    virtual void setMicroSteps(uint32_t ms) = 0;
    virtual void setMaxCurrent(uint32_t mA) = 0;

    virtual uint32_t getMicroSteps() const = 0;
    virtual uint32_t getMaxCurrent() const = 0;
    virtual bool getDirectionPinState() const = 0;
    virtual bool getEnabled() const = 0;

    virtual uint64_t get_arr() const = 0;
    virtual uint64_t get_tmp() const = 0;
};
