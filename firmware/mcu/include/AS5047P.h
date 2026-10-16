#pragma once
#include <stdint.h>
#include "spi.h"
#include "gpio.h"

struct AS5047PStatus
{
    uint16_t raw = 0;

    bool ok = false;
    bool spiError = false;
    bool magnetTooWeak = false;
    bool magnetTooStrong = false;
    bool cordicOverflow = false;
    bool offsetCompFinished = false;
    uint32_t duration = 0;

    uint8_t agc = 0;
};

class AS5047P
{
public:
    AS5047P(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin);

    void begin();

    uint16_t readRawAngle();
    int32_t readAngleMilliDeg();

    AS5047PStatus getStatus();

private:
    SPI_HandleTypeDef* spi_;
    GPIO_TypeDef* csPort_;
    uint16_t csPin_;
    AS5047PStatus status_;

    uint16_t readRegister(uint16_t addr);
    uint16_t makeCommand(uint16_t addr);
    bool checkParity(uint16_t value);

    void csLow();
    void csHigh();
};
