#include "AS5047P.h"

AS5047P::AS5047P(
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* csPort,
    uint16_t csPin)
{
    spi_ = spi;
    csPort_ = csPort;
    csPin_ = csPin;
}

void AS5047P::csLow()
{
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
}

void AS5047P::csHigh()
{
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
}

void AS5047P::begin()
{
    csHigh();
}

// ---- Helper ----
bool AS5047P::checkParity(uint16_t value)
{
    // Odd parity Bit 0..14, Bit 15 is Parity-Bit
    uint16_t v = value & 0x7FFF;
    uint8_t count = 0;

    for (int i = 0; i < 15; i++)
        count += (v >> i) & 1;

    uint8_t parityBit = (value >> 15) & 1;

    return ((count + parityBit) & 1) == 0; // odd parity
}

uint16_t AS5047P::makeCommand(uint16_t addr)
{
    uint16_t cmd = 0x4000 | (addr & 0x3FFF); // R/W=1 (read), 14-bit addr

    // Parity Bit 0..14
    uint8_t count = 0;
    for (int i = 0; i < 15; i++)
        count += (cmd >> i) & 1;

    if ((count & 1) == 1) // even
        cmd |= 0x8000;

    return cmd;
}

// ---- Core: Register read (2-Frame-Protocol) ----
uint16_t AS5047P::readRegister(uint16_t addr)
{
    uint32_t start = DWT->CYCCNT;

    status_.spiError = false;

    uint16_t cmd = makeCommand(addr);
    uint16_t rx = 0;

    // Frame 1: Request
    csLow();
    HAL_SPI_TransmitReceive(spi_, (uint8_t*)&cmd, (uint8_t*)&rx, 1, 10);
    csHigh();

    // Frame 2: Response
    uint16_t dummy = 0xFFFF;
    uint16_t data = 0;

    csLow();
    HAL_SPI_TransmitReceive(spi_, (uint8_t*)&dummy, (uint8_t*)&data, 1, 10);
    csHigh();
    
    // Parity check
    if (!checkParity(data))
    {
        status_.spiError = true;
        return 0xFFFF;
    }
    // Error-Flag (Bit 14)
    if (data & (1 << 14))
    {
        status_.spiError = true;
        return 0xFFFF;
    }
    // Measure duration
    uint32_t end = DWT->CYCCNT;
    uint32_t us = (end - start) / (SystemCoreClock / 1000000);
    status_.duration = us;
    return data & 0x3FFF; // 14-bit payload
}

// ---- Public API ----
uint16_t AS5047P::readRawAngle()
{
    return readRegister(0x3FFF);
}

int32_t AS5047P::readAngleMilliDeg()
{
    uint16_t raw = readRawAngle();
    if (raw == 0xFFFF) {
        return -1;
    }
    int32_t angle_mdeg = (int32_t)((int64_t)raw * 360000LL / 16384LL);

    return angle_mdeg;
}

AS5047PStatus AS5047P::getStatus()
{
    uint16_t d = readRegister(0x3FFC); // DIAAGC

    status_.raw = d;

    status_.agc = d & 0x00FF;

    status_.magnetTooWeak   = (d >> 11) & 0x01;
    status_.magnetTooStrong = (d >> 10) & 0x01;
    status_.cordicOverflow  = (d >> 9) & 0x01;
    status_.offsetCompFinished = (d >> 8) & 0x01;

    status_.ok = 
        !status_.spiError &&
        !status_.magnetTooWeak &&
        !status_.magnetTooStrong &&
        !status_.cordicOverflow &&
        status_.offsetCompFinished;

    return status_;
}
