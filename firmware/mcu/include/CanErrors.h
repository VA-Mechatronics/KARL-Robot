#pragma once
#include <stdint.h>

struct CanErrorInfo
{
    uint32_t state = 0;
    uint32_t error = 0;
};
