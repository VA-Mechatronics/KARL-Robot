#pragma once
#include <stdint.h>
#include <string.h>

class UsbCdc
{
public:
    UsbCdc() = default;

    bool send(const char* text, uint16_t len = 0);
    static void onReceive(uint8_t* buf, uint32_t len);

    int32_t get_vel(void);
    bool get_enable(void);

private:
    bool transmitBlocking(const uint8_t* data, uint16_t len);
    
    static int32_t vel_;
    static bool enable_;
};
