#ifndef BOOTLOADER_JUMP_H
#define BOOTLOADER_JUMP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Checks the RTC backup register at system startup.
 *         Must be called FIRST in the main() function.
 */
void Bootloader_CheckAndJump(void);

/**
 * @brief  Configure the “Bytes” option once so that the BOOT0 pin is ignored.
 *         Should be called before the main loop.
 */
void Bootloader_InitOptionBytes(void);

/**
 * @brief  Triggers a software reset to switch to the DFU bootloader in a controlled manner.
 *         Can be called later via your USB command instead of the delay.
 */
void Bootloader_TriggerDFU(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif // BOOTLOADER_JUMP_H
