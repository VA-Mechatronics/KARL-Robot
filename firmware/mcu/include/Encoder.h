#pragma once
#include <stdint.h>
#include "AS5047P.h"

struct EncoderStatus
{
    int32_t position_mdeg;      // multiturn absolute position
    int32_t velocity_mdeg_s;    // velocity
    int32_t angle_mdeg;         // single turn angle
    int32_t turns;
};

class Encoder
{
public:
    Encoder(AS5047P& hw, int32_t gr_scaled, int32_t offset);

    void update(uint32_t dt_us);

    int32_t getMotorPosition() const;
    int32_t getPosition() const;
    int32_t getMotorVelocity() const;
    int32_t getVelocity() const;
    int32_t getOffset() const;
    int32_t getGr() const;

    void setOffset(int32_t offset);
    void setZero();
    void setGr(int32_t gr);

    EncoderStatus getStatus() const;

private:
    AS5047P& hw_;

    int32_t gr_;

    int32_t last_angle_ = 0;        // 0..360000

    int32_t turns_ = 0;

    int32_t offset_ = 0;

    int32_t velocity_ = 0;
    int32_t velocity_filt_ = 0;
};