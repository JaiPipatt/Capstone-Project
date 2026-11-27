#include "bootloader.h"
#include "stm32f4xx_hal.h"

/* System memory (ROM bootloader) of STM32F401 / STM32F411 */
#define SYSMEM_BASE        0x1FFF0000UL
#define BOOT_MAGIC         0xB00710ADUL

/* Kept in .noinit: NOT cleared by the startup code, survives a software reset. */
static volatile uint32_t g_boot_magic __attribute__((section(".noinit")));
static volatile uint8_t  g_boot_requested = 0U;

void Bootloader_JumpIfRequested(void)
{
    if (g_boot_magic != BOOT_MAGIC) {
        return;                                   /* normal start */
    }
    g_boot_magic = 0U;                            /* next reset = normal start */

    /* Chip just came out of reset: clocks are on HSI, no interrupts running. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();       /* map ROM bootloader at 0x0 */

    const uint32_t boot_sp    = *(volatile uint32_t *)(SYSMEM_BASE + 0U);
    const uint32_t boot_reset = *(volatile uint32_t *)(SYSMEM_BASE + 4U);

    __set_MSP(boot_sp);
    ((void (*)(void))boot_reset)();               /* never returns */

    for (;;) { }
}

void Bootloader_Request(void)
{
    g_boot_requested = 1U;
}

void Bootloader_Service(void)
{
    if (g_boot_requested == 0U) {
        return;
    }
    HAL_Delay(100);                               /* let the USB request finish */
    g_boot_magic = BOOT_MAGIC;
    __DSB();
    NVIC_SystemReset();                           /* never returns */
}
