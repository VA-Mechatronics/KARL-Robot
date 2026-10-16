#pragma once
#include "CanErrors.h"
#include <stdio.h>

inline const char* decodeCanError(const CanErrorInfo& e)
{
    static char buf[128];
    snprintf(
        buf, sizeof(buf),
        "CAN State=0x%08lX Error=0x%08lX\n",
        (unsigned long)e.state,
        (unsigned long)e.error
    );
    return buf;
}
