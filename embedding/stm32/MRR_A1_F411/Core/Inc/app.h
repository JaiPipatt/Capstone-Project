#ifndef APP_H
#define APP_H

/* Call once in main.c, inside USER CODE BEGIN 2 (after all MX_..._Init). */
void App_Init(void);

/* Call repeatedly inside the while(1) loop (USER CODE BEGIN 3). */
void App_MainLoop(void);

/* Called from CDC_Receive_FS() in USB_DEVICE/App/usbd_cdc_if.c (USB interrupt).
 * Lets you drive the board from the PC over the USB-C cable (virtual COM port),
 * with the same frames as the ESP32 UART link. */
#include <stdint.h>
void App_UsbRx(const uint8_t *data, uint32_t len);

#endif /* APP_H */
