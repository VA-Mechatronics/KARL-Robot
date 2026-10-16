#pragma once

#include "StepperDriver.h"

class TMC2209 : public StepperDriver
{
public:
    TMC2209(
        GPIO_TypeDef* enPort, uint16_t enPin,
        GPIO_TypeDef* dirPort, uint16_t dirPin,
        GPIO_TypeDef* stepPort, uint16_t stepPin,
        GPIO_TypeDef* ms1Port, uint16_t ms1Pin,
        GPIO_TypeDef* ms2Port, uint16_t ms2Pin,
        GPIO_TypeDef* sprdPort, uint16_t sprdPin,
        GPIO_TypeDef* pdnPort, uint16_t pdnPin
    );

    void begin() override;
    DriverStatus getStatus() override;

    void enable(bool en) override;
    void setVelocity(int64_t vel_mdeg_s) override;

    void setMicroSteps(uint32_t ms) override;
    void setMaxCurrent(uint32_t mA) override;

    uint32_t getMicroSteps() const override;
    uint32_t getMaxCurrent() const override;
    bool getDirectionPinState() const override;
    bool getEnabled() const override;

    uint64_t get_arr() const override;
    uint64_t get_tmp() const override;

private:
    GPIO_TypeDef* enPort_;
    uint16_t enPin_;
    GPIO_TypeDef* dirPort_;
    uint16_t dirPin_;
    GPIO_TypeDef* stepPort_;
    uint16_t stepPin_;
    GPIO_TypeDef* ms1Port_;
    uint16_t ms1Pin_;
    GPIO_TypeDef* ms2Port_;
    uint16_t ms2Pin_;
    GPIO_TypeDef* sprdPort_;
    uint16_t sprdPin_;
    GPIO_TypeDef* pdnPort_;
    uint16_t pdnPin_;

    DriverStatus status_;
    uint32_t microsteps_ = 16;
    uint32_t stepsPerRev_ = 200;
    uint32_t maxCurrent_ = 0;
    
    uint64_t arr_ = 0;
    uint64_t tmp_ = 0;
};
