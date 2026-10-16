#include "Encoder.h"

static constexpr int32_t FULL_ROTATION = 360000;
static constexpr int32_t HALF_ROTATION = 180000;

Encoder::Encoder(AS5047P& hw, int32_t gr_scaled, int32_t offset)
    : hw_(hw), gr_(gr_scaled), offset_(offset)
{
    last_angle_ = hw_.readAngleMilliDeg();
}

void Encoder::update(uint32_t dt_us) {
    int32_t angle = hw_.readAngleMilliDeg();

    // delta angle with wrap handling
    int32_t delta = angle - last_angle_;

    // multiturn
    if (delta > HALF_ROTATION)
    {
        delta -= FULL_ROTATION;
        turns_--;
    }
    else if (delta < -HALF_ROTATION)
    {
        delta += FULL_ROTATION;
        turns_++;
    }

    // velocity
    int32_t vel_raw = (delta * 1000LL) / dt_us;      // mdeg/s

    // IIR filter
    velocity_filt_ += (vel_raw - velocity_filt_) / 5;
    velocity_ = velocity_filt_;
    last_angle_ = angle;
}

int32_t Encoder::getMotorPosition() const {
    return turns_ * FULL_ROTATION + last_angle_;
}

int32_t Encoder::getPosition() const {
    int32_t motor = getMotorPosition();
    int64_t output = (static_cast<int64_t>(motor) * 1000LL) / gr_;
    return static_cast<int32_t>(output - offset_);
}

int32_t Encoder::getMotorVelocity() const { return velocity_; }

int32_t Encoder::getVelocity() const {
    int32_t motor = getMotorVelocity();
    int64_t output = (static_cast<int64_t>(motor) * 1000LL) / gr_;
    return static_cast<int32_t>(output);
}

int32_t Encoder::getOffset() const { return offset_; }

void Encoder::setOffset(int32_t offset) { offset_ = offset; }

void Encoder::setZero() {
    int32_t motor_pos = getMotorPosition();
    int64_t output = (static_cast<int64_t>(motor_pos) * 1000LL) / gr_;
    offset_ = static_cast<int32_t>(output);
}

void Encoder::setGr(int32_t gr_scaled) {
    if (gr_ == gr_scaled || gr_scaled == 0) return;

    int32_t pos_before = getPosition();
    int32_t motor_pos = getMotorPosition();
    gr_ = gr_scaled;
    int64_t new_pos = (static_cast<int64_t>(motor_pos) * 1000LL) / gr_;
    offset_ = static_cast<int32_t>(new_pos - pos_before);
}

int32_t Encoder::getGr() const { return gr_; }

EncoderStatus Encoder::getStatus() const {
    EncoderStatus s;

    s.position_mdeg = getPosition();
    s.velocity_mdeg_s = velocity_;
    s.angle_mdeg = last_angle_;
    s.turns = turns_;

    return s;
}