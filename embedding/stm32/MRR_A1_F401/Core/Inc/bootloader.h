#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>

/* ----------------------------------------------------------------------
 * Enter the STM32 built-in USB DFU bootloader WITHOUT pressing BOOT0/NRST.
 *
 * How it works:
 *   1. The PC opens the USB serial port at 1200 baud (flash_dfu.sh does it).
 *   2. usbd_cdc_if.c sees baud = 1200 and calls Bootloader_Request().
 *   3. The main loop calls Bootloader_Service(): it writes a magic number in
 *      RAM that survives a reset, then resets the chip.
 *   4. At the very start of main(), Bootloader_JumpIfRequested() sees the
 *      magic number and jumps to the ROM bootloader (system memory 0x1FFF0000).
 *   5. The board appears as "STM32 BOOTLOADER" (USB DFU, 0483:df11).
 * -------------------------------------------------------------------- */

/* Call FIRST in main(), before HAL_Init(). */
void Bootloader_JumpIfRequested(void);

/* Call from the USB CDC callback (interrupt context): only sets a flag. */
void Bootloader_Request(void);

/* Call from the main loop: performs the reset when requested. */
void Bootloader_Service(void);

#endif /* BOOTLOADER_H */
