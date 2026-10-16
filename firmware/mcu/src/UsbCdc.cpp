#include "UsbCdc.h"
#include "usbd_cdc_if.h"
#include "stm32g4xx_hal.h"
#include "bootloader_jump.h"
#include "Io.h"

int32_t UsbCdc::vel_ = 0;
bool UsbCdc::enable_ = true;

bool UsbCdc::send(const char* text, uint16_t len)
{
    if(len == 0)
        len = strlen(text);

    return transmitBlocking((const uint8_t*)text, len);
}

bool UsbCdc::transmitBlocking(const uint8_t* data, uint16_t len)
{
    static char buf[256];

    uint32_t timeout = HAL_GetTick() + 10; // 10ms Timeout

    // Timestamp
    int prefix_len = snprintf(buf, sizeof(buf), "%lu - ", HAL_GetTick());
    //int prefix_len = snprintf(buf, sizeof(buf), "");

    // Check size
    if (prefix_len < 0 || (size_t)prefix_len >= sizeof(buf)) {
        return false; // Timestamp error → not sent
    }

    // Append payload
    uint16_t copy_len = len;
    if ((size_t)prefix_len + copy_len > sizeof(buf)) {
        copy_len = sizeof(buf) - prefix_len;
    }
    memcpy(buf + prefix_len, data, copy_len);

    uint16_t total_len = prefix_len + copy_len;

    // Send
    while (CDC_Transmit_FS((uint8_t*)buf, total_len) == USBD_BUSY)
    {
        if (HAL_GetTick() > timeout)
            return false; // Timeout → not sent
    }

    return true;
}

void UsbCdc::onReceive(uint8_t* buf, uint32_t len)
{
    // Check whether the word “bootloader” was received
    if (len >= 10 && memcmp(buf, "bootloader", 10) == 0)
    {
        // Immediately trigger a clean reset into DFU mode
        Bootloader_TriggerDFU();
    }

    // NOTE: If you want to process regular data in the program later,
    // you can copy the buffer from this point into a separate circular buffer.
    if (len >= 4 && memcmp(buf, "VEL ", 4) == 0)
    {
        vel_ = atoi((char*)(buf + 4));
    }

    if (len >= 3 && memcmp(buf, "EN ", 3) == 0)
    {
        enable_ = atoi((char*)(buf + 3));
    }
}

extern "C" void UsbCdc_onReceive(uint8_t* buf, uint32_t len)
{
    UsbCdc::onReceive(buf, len);
}

int32_t UsbCdc::get_vel(void) {
    return vel_;
}

bool UsbCdc::get_enable(void) {
    return enable_;
}
