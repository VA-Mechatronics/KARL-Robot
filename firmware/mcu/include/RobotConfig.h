#pragma once

#include <stdint.h>

// 1. Definition of the ID for this specific MCU
// Before flashing, you only need to change this one number (1 through 6):
#define OWN_NODE_ID 3

// 2. Structure for all ID-specific settings
struct MCUConfig {
    int32_t home_offset;
    int32_t mot_gr;
    int32_t enc_gr;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t kv;
    int32_t guiding_gain;
    int32_t guiding_deadzone;
    int32_t error_limit;
};

// 3. Central table for all 6 MCUs
static const MCUConfig ROBOT_NODE_SETTINGS[7] = {
    // ID 1: Base
    { 147000, -18000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000}, 
    // ID 2: Upper Arm
    { -140000, 26000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000},  
    // ID 3: Lower Arm
    { -150000, 31000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000},  
    // ID 4: Wrist 1
    { 112000, 32000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000}, 
    // ID 5: Wrist 2
    { -43000, -48000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000}, 
    // ID 6: Wrist 3
    { 0, -20000, -20000, 1000, 0, 0, 1000, 50000, 1000, 50000},
    // ID 7: Gripper
    { 0, 1000, -1000, 1000, 0, 0, 1000, 50000, 1000, 50000}  
};


#if (OWN_NODE_ID == 5 || OWN_NODE_ID == 6)
    #define DRIVER_USE_TMC2209
#endif

//#define DRIVER_USE_TMC2209

#define ACTIVE_CONFIG (ROBOT_NODE_SETTINGS[OWN_NODE_ID - 1])
