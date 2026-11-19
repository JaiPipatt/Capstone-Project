#ifndef STM32_LINK_H
#define STM32_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "protocol.h"

/* Start UART1 + two FreeRTOS tasks:
 *   rx task: parses frames from the STM32 and keeps the latest MSG_STATE
 *   tx task: resends the latest command every LINK_CMD_PERIOD_MS            */
void stm32_link_init(void);

/* Replace the command that the tx task keeps sending (thread-safe). */
void stm32_link_set_command(const CmdPayload *cmd);

/* Copy of the newest state. Returns false if none received yet.
 * *age_ms = how old it is. */
bool stm32_link_get_state(StatePayload *out, uint32_t *age_ms);

#endif /* STM32_LINK_H */
