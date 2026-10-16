#include "TMC5160.h"

// General configuration
#define REG_GCONF          0x00
#define REG_GSTAT          0x01
#define REG_IFCNT          0x02

// Current control
#define REG_IHOLD_IRUN     0x10
#define REG_TPOWERDOWN     0x11
#define REG_TSTEP          0x12

// Velocity thresholds
#define REG_TPWMTHRS       0x13
#define REG_TCOOLTHRS      0x14
#define REG_THIGH          0x15

// Current scaling
#define REG_GLOBALSCALER   0x0B

// Driver configuration
#define REG_CHOPCONF       0x6C
#define REG_COOLCONF       0x6D
#define REG_DRV_STATUS     0x6F
#define REG_PWMCONF        0x70
#define REG_PWM_SCALE      0x71
#define REG_PWM_AUTO       0x72

#define REG_IOIN           0x04
#define REG_DRVSTATUS      0x6F

static const uint64_t sysClock = 16000000ULL;

TMC5160::TMC5160(
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* csPort, uint16_t csPin,
    GPIO_TypeDef* enPort, uint16_t enPin,
    GPIO_TypeDef* dirPort, uint16_t dirPin,
    GPIO_TypeDef* stepPort, uint16_t stepPin)
{
    spi_ = spi;
    csPort_ = csPort;
    csPin_ = csPin;
    enPort_ = enPort;
    enPin_ = enPin;
    dirPort_ = dirPort;
    dirPin_ = dirPin;
    stepPort_ = stepPort;
    stepPin_ = stepPin;
}

void TMC5160::csLow()
{
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
}

void TMC5160::csHigh()
{
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
}

void TMC5160::write(uint8_t addr, uint32_t value)
{
    uint8_t tx[5];

    tx[0] = addr | 0x80;
    tx[1] = (value >> 24) & 0xFF;
    tx[2] = (value >> 16) & 0xFF;
    tx[3] = (value >> 8) & 0xFF;
    tx[4] = (value) & 0xFF;

    csLow();
    if (HAL_SPI_Transmit(spi_, tx, 5, 10) != HAL_OK)
    {
        status_.spiError = true;
        status_.ok = false;
    }
    else
    {
        status_.spiError = false;
        status_.ok = true;
    }
    csHigh();
}

void TMC5160::writeRegister(uint8_t addr, uint32_t value)
{
    write(addr, value);
}

uint32_t TMC5160::readRegister(uint8_t addr)
{
    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    tx[0] = addr & 0x7F; // READ bit = 0

    csLow();
    HAL_SPI_TransmitReceive(spi_, tx, rx, 5, 10);
    csHigh();

    // second transaction (TMC requires dummy cycle)
    tx[0] = addr & 0x7F;

    csLow();
    HAL_SPI_TransmitReceive(spi_, tx, rx, 5, 10);
    csHigh();

    uint32_t value =
        (rx[1] << 24) |
        (rx[2] << 16) |
        (rx[3] << 8)  |
        (rx[4]);

    return value;
}

void TMC5160::begin()
{
    uint32_t gconf =
          (1 << 7)      // en_pwm_mode -> StealthChop active
        | (1 << 6)      // pdn_disable -> SPI
        | (0 << 2)      // shaft -> DIR Pin normal
        | (0 << 8);     // direct_mode off
    writeRegister(REG_GCONF, gconf);

    // Reduce max global max Current, 255 = 100%
    writeRegister(REG_GLOBALSCALER, 255);

    uint32_t chopconf =
          (4 << 0)       // TOFF = 4 (must be >2)
        | (0 << 7)       // HEND (almost irrelevant for Stealth)
        | (0 << 11)      // HSTRT (almost irrelevant for Stealth)
        | (2 << 15)      // TBL = 2 (36 clocks)
        | (4 << 24)      // MRES = 4 -> 1/16
        | (1 << 28);     // INTPOL = 256 interpolation
    writeRegister(REG_CHOPCONF, chopconf);

    uint32_t ihold_irun =
          (10  << 0)     // IHOLD
        | (10 << 8)      // IRUN
        | (6  << 16);    // IHOLDDELAY
    writeRegister(REG_IHOLD_IRUN, ihold_irun);

    // extremely high -> always use StealthChop
    writeRegister(REG_TPWMTHRS, 0x000FFFFF);

    uint32_t pwmconf =
          (36 << 0)      // PWM_OFS low = more aggressiv
        | (5 << 8)       // PWM_GRAD low = more aggressiv
        | (1 << 18)      // PWM_AUTOSCALE
        | (1 << 19);     // PWM_AUTOGRAD
    writeRegister(REG_PWMCONF, pwmconf);

    // CoolStep deactivate
    writeRegister(REG_TCOOLTHRS, 1);

    // StallGuard deactivate
    writeRegister(REG_COOLCONF, 1);

    TIM1->PSC = 0;
    TIM1->EGR |= TIM_EGR_UG; // Force update
}

DriverStatus TMC5160::getStatus()
{
    DriverStatus st;

    st.drvStatus = readRegister(REG_DRVSTATUS);
    st.gstat     = readRegister(REG_GSTAT);
    st.ioin      = readRegister(REG_IOIN);
    st.pwmScale  = readRegister(REG_PWM_SCALE);
    st.pwmAuto   = readRegister(REG_PWM_AUTO);

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

void TMC5160::setMicroSteps(uint32_t microsteps) {
    microsteps_ = microsteps;

    uint32_t mres = 0;

    switch (microsteps)
    {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        default:   mres = 4; break;
    }

    uint32_t chopconf = readRegister(REG_CHOPCONF);

    chopconf &= ~(0xF << 24);
    chopconf |= (mres << 24);

    writeRegister(REG_CHOPCONF, chopconf);
}

uint32_t TMC5160::getMicroSteps() const { return microsteps_; }

void TMC5160::setMaxCurrent(uint32_t mA) {
    maxCurrent_ = mA;

    // Simplified scaling (depends on Rsense!)
    uint32_t irun = (mA * 31) / 1500; // 1.5A max
    if (irun > 31) irun = 31;
    uint32_t ihold = irun;

    uint32_t ihold_irun =
          (ihold << 0)    // IHOLD
        | (irun << 8)     // IRUN
        | (1 << 16);      // IHOLDDELAY

    writeRegister(REG_IHOLD_IRUN, ihold_irun);
}

uint32_t TMC5160::getMaxCurrent() const { return maxCurrent_; }

bool TMC5160::getDirectionPinState() const {
    return HAL_GPIO_ReadPin(dirPort_, dirPin_) == GPIO_PIN_SET;
}

void TMC5160::enable(bool en)
{
    if(en)
        HAL_GPIO_WritePin(enPort_, enPin_, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(enPort_, enPin_, GPIO_PIN_SET);
}

bool TMC5160::getEnabled() const {
    return HAL_GPIO_ReadPin(enPort_, enPin_) == GPIO_PIN_RESET;
}

void TMC5160::setVelocity(int64_t vel_mdeg_s)
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

uint64_t TMC5160::get_arr() const { return arr_; }
uint64_t TMC5160::get_tmp() const { return tmp_; }