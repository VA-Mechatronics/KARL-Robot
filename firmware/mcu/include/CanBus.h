#pragma once
#include "CanFrame.h"
#include "CanErrors.h"
#include "fdcan.h"

class CanBus
{
public:
    explicit CanBus(FDCAN_HandleTypeDef& handle);

    bool getReceived(CanFrame& frame);
    bool send(uint8_t onwNodeId, const PayloadMessage& status);
    bool getError(CanErrorInfo& info);
    void onRxInterrupt(FDCAN_RxHeaderTypeDef& rxHeader, uint8_t* data);
    void configureCanFilters(FDCAN_HandleTypeDef* hfdcan, uint8_t ownNodeId);

private:
    FDCAN_HandleTypeDef& h_;

    static constexpr int BUF_SIZE = 32;

    CanFrame rxBuffer[BUF_SIZE];
    volatile uint16_t writeIdx{0};
    volatile uint16_t readIdx{0};
    static inline uint8_t dlcToBytes(uint32_t dlc);
};
