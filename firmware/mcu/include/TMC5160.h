#pragma once

#include "StepperDriver.h"
#include "spi.h"

class TMC5160 : public StepperDriver
{
public:
    TMC5160(
        SPI_HandleTypeDef* spi,
        GPIO_TypeDef* csPort, uint16_t csPin,
        GPIO_TypeDef* enPort, uint16_t enPin,
        GPIO_TypeDef* dirPort, uint16_t dirPin,
        GPIO_TypeDef* stepPort, uint16_t stepPin
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
    SPI_HandleTypeDef* spi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
    GPIO_TypeDef* enPort_;
    uint16_t enPin_;
    GPIO_TypeDef* dirPort_;
    uint16_t dirPin_;
    GPIO_TypeDef* stepPort_;
    uint16_t stepPin_;

    DriverStatus status_;
    uint32_t microsteps_ = 16;
    uint32_t stepsPerRev_ = 200;
    uint32_t maxCurrent_ = 1200;

    void writeRegister(uint8_t addr, uint32_t value);
    void write(uint8_t addr, uint32_t value);
    uint32_t readRegister(uint8_t addr);

    void csLow();
    void csHigh();
    
    uint64_t arr_ = 0;
    uint64_t tmp_ = 0;
};
