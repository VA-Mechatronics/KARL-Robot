#include "bootloader_jump.h"
#include "stm32g4xx_hal.h"

#define STM32_SYSTEM_MEMORY_ADDR 0x1FFF0000

void Bootloader_CheckAndJump(void)
{
    // Enable access to the backup registers
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_ENABLE(); 

    // Check whether the bootloader flag is enabled
    if (TAMP->BKP0R == 0xDEADBEEF) {
        TAMP->BKP0R = 0; // Clear flag
        
        // The bootloader absolutely requires the internal HSI16 clock
        RCC->CR |= RCC_CR_HSION;
        while((RCC->CR & RCC_CR_HSIRDY) == 0);
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_HSI;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
        
        // Reset peripherals and HAL
        HAL_RCC_DeInit();
        HAL_DeInit();
        
        // Stop SysTick
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL  = 0;
        
        // Interrupts global off
        __disable_irq();
        
        // Clear all NVIC-Flags
        for (int i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }
        
        // System Flash
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
        
        // Retrieve addresses, set the stack pointer, and jump
        uint32_t jumpAddress = *(__IO uint32_t*)(STM32_SYSTEM_MEMORY_ADDR + 4);
        void (*jumpToBootloader)(void) = (void (*)(void))jumpAddress;
        
        __set_MSP(*(__IO uint32_t*)STM32_SYSTEM_MEMORY_ADDR);
        jumpToBootloader();
        
        while(1); // Failsafe
    }
}

void Bootloader_InitOptionBytes(void)
{
    FLASH_OBProgramInitTypeDef OBInit;
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    HAL_FLASHEx_OBGetConfig(&OBInit);

    // If the hardware pin is still active (the bit is 1), we reprogram it
    if ((OBInit.USERConfig & FLASH_OPTR_nSWBOOT0) != 0) {
        OBInit.OptionType = OPTIONBYTE_USER;
        OBInit.USERType   = OB_USER_nSWBOOT0; 
        
        // Ignore pin, force Flash boot
        OBInit.USERConfig &= ~FLASH_OPTR_nSWBOOT0;
        OBInit.USERConfig |= FLASH_OPTR_nBOOT0;
        
        if (HAL_FLASHEx_OBProgram(&OBInit) == HAL_OK) {
            // Triggers a hard reset. The bootloader flag is briefly lost,
            // but since we're still in the application, it doesn't matter.
            HAL_FLASH_OB_Launch(); 
        }
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

void Bootloader_TriggerDFU(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_ENABLE();

    TAMP->BKP0R = 0xDEADBEEF; // Write the magic value
    HAL_NVIC_SystemReset();   // Trigger a true hardware reset

    while(1); // Wait for the reset
}
