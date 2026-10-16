#include "CanBus.h"

CanBus::CanBus(FDCAN_HandleTypeDef& handle)
    : h_(handle)
{
    // e.g. dynamic filters here
}

bool CanBus::getReceived(CanFrame& frame)
{
    uint16_t currentRead = readIdx;
    uint16_t currentWrite = writeIdx;
    if (currentRead == currentWrite) {
        return false;
    }

    frame = rxBuffer[currentRead];

    readIdx = (currentRead + 1) % BUF_SIZE;

    return true;
}

bool CanBus::send(uint8_t ownNodeId, const PayloadMessage& status)
{
    FDCAN_TxHeaderTypeDef header{};
    
    // Calculate ID: e.g., group 0x300 + axis 1 = 0x301
    header.Identifier = 0x300 | (ownNodeId & 0x0F);
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_20;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_ON;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    // Send from RAM-Struct
    return HAL_FDCAN_AddMessageToTxFifoQ(&h_, &header, const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&status))) == HAL_OK;
}

bool CanBus::getError(CanErrorInfo& info)
{
    info.state = HAL_FDCAN_GetState(&h_);
    info.error = HAL_FDCAN_GetError(&h_);
    return true;
}

void CanBus::onRxInterrupt(FDCAN_RxHeaderTypeDef& rxHeader, uint8_t* rxData)
{
    uint8_t reallen = dlcToBytes(rxHeader.DataLength);
    uint8_t bytesToCopy = (reallen > 64) ? 64 : reallen;

    uint16_t currentWrite = writeIdx;
    uint16_t nextWrite = (currentWrite + 1) % BUF_SIZE;

    if (nextWrite == readIdx) {
        return;
    }

    CanFrame& msg = rxBuffer[currentWrite];
    msg.id = rxHeader.Identifier;
    msg.dlc = rxHeader.DataLength;

    for (int i = 0; i < bytesToCopy; i++)
        msg.data[i] = rxData[i];

    writeIdx = nextWrite;
}

uint8_t CanBus::dlcToBytes(uint32_t dlc)
{
    static const uint8_t table[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8,   
        12, 16, 20, 24, 32, 48, 64   
    };
    if (dlc < 16) return table[dlc];
    return 0;
}

void CanBus::configureCanFilters(FDCAN_HandleTypeDef* hfdcan, uint8_t ownNodeId)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // --- FILTER 0: BROADCASTS (E-Stopp 0x000 & Motion 0x100) ---
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;                     // First filter element
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;      // Mask-Mode 
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 
    sFilterConfig.FilterID1 = 0x000;                   // ID 1: E-Stopp
    sFilterConfig.FilterID2 = 0x100;                   // ID 2: Motion Broadcast

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    // --- FILTER 1: CUSTOM CONFIG- & COMMAND-MESSAGES (Exact Match) ---
    uint32_t ownConfigId = 0x200 | (ownNodeId & 0x0F);

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;                     // Second filter element
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;      // DUAL allows two exact IDs without mask evaluation
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = ownConfigId;             // First exact ID (Config for this axis)
    sFilterConfig.FilterID2 = ownConfigId;             // Second ID (Chosen for convenience, since we only need one)

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);

    // --- GLOBAL CONFIGURATION ---
    // Defines what happens to messages that do not match any of the filters above.
    // Parameter 2: Standard IDs -> REJECT
    // Parameter 3: Extended IDs -> REJECT
    // Parameter 4: Reject remote frames for Standard
    // Parameter 5: Reject remote frames for Extended
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}