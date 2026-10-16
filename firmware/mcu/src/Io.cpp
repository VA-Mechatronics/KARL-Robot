#include "Io.h"

void red_blink_routine(uint8_t count, uint32_t delay) {
    HAL_GPIO_WritePin(GPIO_Output_RED_GPIO_Port, GPIO_Output_RED_Pin, GPIO_PIN_RESET);
    for (int i=0; i<count; i++) {
        HAL_GPIO_TogglePin(GPIO_Output_RED_GPIO_Port, GPIO_Output_RED_Pin);
        HAL_Delay(delay);
    }
    HAL_GPIO_WritePin(GPIO_Output_RED_GPIO_Port, GPIO_Output_RED_Pin, GPIO_PIN_RESET);
}