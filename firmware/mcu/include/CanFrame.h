#pragma once
#include <stdint.h>
#include <array>
#include <stdio.h>

struct __attribute__((packed)) PayloadMessage {
    uint8_t msg_type = 0;
    uint8_t write = 0;
    int16_t echo = 0;
    int32_t p1 = 0;
    int32_t p2 = 0;
    int32_t p3 = 0;
    int32_t p4 = 0;
};

struct __attribute__((packed)) AxisTarget {
    int32_t set_pos;
    int32_t set_vel;
};

struct __attribute__((packed)) GlobalMotionBroadcast {
    uint8_t command;
    uint8_t sync_counter;
    uint16_t timestamp;
    AxisTarget axes[6];
};

struct CanFrame
{
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 64> data{};
};
