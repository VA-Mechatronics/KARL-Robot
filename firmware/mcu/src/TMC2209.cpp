#include "TMC2209.h"

static const uint64_t sysClock = 16000000ULL;

TMC2209::TMC2209(
    GPIO_TypeDef* enPort, uint16_t enPin,
    GPIO_TypeDef* dirPort, uint16_t dirPin,
    GPIO_TypeDef* stepPort, uint16_t stepPin,
    GPIO_TypeDef* ms1Port, uint16_t ms1Pin,
    GPIO_TypeDef* ms2Port, uint16_t ms2Pin,
    GPIO_TypeDef* sprdPort, uint16_t sprdPin,
    GPIO_TypeDef* pdnPort, uint16_t pdnPin)
{
    enPort_ = enPort;
    enPin_ = enPin;
    dirPort_ = dirPort;
    dirPin_ = dirPin;
    stepPort_ = stepPort;
    stepPin_ = stepPin;
    ms1Port_ = ms1Port;
    ms1Pin_ = ms1Pin;
    ms2Port_ = ms2Port;
    ms2Pin_ = ms2Pin;
    sprdPort_ = sprdPort;
    sprdPin_ = sprdPin;
    pdnPort_ = pdnPort;
    pdnPin_ = pdnPin;
}

void TMC2209::begin()
{
    // --- HARDWARE-RECONFIGURATION SPI-PINS TO GPIO-OUTPUTS ---
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Confgiguration: Digital Output, Push-Pull, no Pull-Up/Down, low Freq
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // 1. MS1 (old MISO)
    GPIO_InitStruct.Pin = ms1Pin_;
    HAL_GPIO_Init(ms1Port_, &GPIO_InitStruct);

    // 2. MS2 (old SCK)
    GPIO_InitStruct.Pin = ms2Pin_;
    HAL_GPIO_Init(ms2Port_, &GPIO_InitStruct);

    // 3. SPREAD (old CS)
    GPIO_InitStruct.Pin = sprdPin_;
    HAL_GPIO_Init(sprdPort_, &GPIO_InitStruct);

    // 4. PDN_UART (old MOSI)
    GPIO_InitStruct.Pin = pdnPin_;
    HAL_GPIO_Init(pdnPort_, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sprdPort_, sprdPin_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pdnPort_, pdnPin_, GPIO_PIN_RESET);

    TIM1->PSC = 0;
    TIM1->EGR |= TIM_EGR_UG; // Force update
}

DriverStatus TMC2209::getStatus()
{
    DriverStatus st;

    st.drvStatus = 0;
    st.gstat     = 0;
    st.ioin      = 0;
    st.pwmScale  = 0;
    st.pwmAuto   = 0;

    st.ok = true;
    st.spiError = false;

    // DRV_STATUS decoding
    st.overTempPrewarn = st.drvStatus & (1 << 26);
    st.overTemp        = st.drvStatus & (1 << 25);
    st.shortToGND      = st.drvStatus & (1 << 24);
    st.undervoltage    = st.drvStatus & (1 << 13);
    st.openLoad        = st.drvStatus & (1 << 12);
    st.standstill      = st.drvStatus & (1 << 31);

    // 0=no current, 1=minimal, 31=maximal relative IRUN
    st.currentScale = (st.drvStatus >> 16) & 0x1F;

    // 1000 = low load, 0 = high load
    st.sgResult = st.drvStatus & 0x3FF;

    return st;
}

void TMC2209::setMicroSteps(uint32_t microsteps) {
    microsteps_ = microsteps;

    switch (microsteps)
    {
        case 64:
            HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_SET);
            break;
        case 32:
            HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_RESET);
            break;
        case 16:
            HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_SET);
            break;
        case 8: 
            HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_RESET);
            break;
        default: 
            HAL_GPIO_WritePin(ms1Port_, ms1Pin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ms2Port_, ms2Pin_, GPIO_PIN_SET);
            break;
    }
}

uint32_t TMC2209::getMicroSteps() const { return microsteps_; }

void TMC2209::setMaxCurrent(uint32_t mA) {
    maxCurrent_ = mA;
}

uint32_t TMC2209::getMaxCurrent() const { return maxCurrent_; }

bool TMC2209::getDirectionPinState() const {
    return HAL_GPIO_ReadPin(dirPort_, dirPin_) == GPIO_PIN_SET;
}

void TMC2209::enable(bool en)
{
    if(en)
        HAL_GPIO_WritePin(enPort_, enPin_, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(enPort_, enPin_, GPIO_PIN_SET);
}

bool TMC2209::getEnabled() const {
    return HAL_GPIO_ReadPin(enPort_, enPin_) == GPIO_PIN_RESET;
}

void TMC2209::setVelocity(int64_t vel_mdeg_s)
{
    if (vel_mdeg_s >= 0)
        HAL_GPIO_WritePin(dirPort_, dirPin_, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(dirPort_, dirPin_, GPIO_PIN_RESET);

    uint64_t abs_vel = (vel_mdeg_s >= 0) ? vel_mdeg_s : -vel_mdeg_s;
    uint64_t denominator = abs_vel * (uint64_t)stepsPerRev_ * (uint64_t)microsteps_;

    if (denominator == 0) {
        TIM1->CCR1 = 0; // Stop PWM-Puls
        return;
    }
    uint64_t arr_raw = (sysClock * 360000ULL) / denominator;
    uint32_t psc = 0;

    // Dynamic prescaler
    if (arr_raw > 65535) {
        psc = (uint32_t)(arr_raw / 65535);
        arr_raw = arr_raw / (uint64_t)(psc + 1);
    }

    if (arr_raw > 65535) arr_raw = 35535;
    if (arr_raw < 10) arr_raw = 10;

    arr_ = arr_raw;
    tmp_ = psc;

    if (TIM1->PSC != psc) {
        TIM1->PSC = psc;
        TIM1->EGR |= TIM_EGR_UG;
    }

    TIM1->ARR = (uint32_t)arr_raw;
    TIM1->CCR1 = (uint32_t)(arr_raw / 2);

    if (!(TIM1->CR1 & TIM_CR1_CEN)) {
        TIM1->CR1 |= TIM_CR1_CEN;
    }
}

uint64_t TMC2209::get_arr() const { return arr_; }
uint64_t TMC2209::get_tmp() const { return tmp_; }
