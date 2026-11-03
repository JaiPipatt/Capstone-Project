#include "stm32_link.h"
#include "robot_config.h"

#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "stm32_link";

static ProtoParser  s_parser;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static CmdPayload   s_cmd;                 /* all zero = every motor MODE_STOP */
static StatePayload s_state;
static TickType_t   s_state_tick;
static bool         s_state_valid;

static void on_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
    if (type == MSG_STATE && len == sizeof(StatePayload)) {
        StatePayload st;
        memcpy(&st, p, sizeof st);

        portENTER_CRITICAL(&s_mux);
        s_state       = st;
        s_state_tick  = xTaskGetTickCount();
        s_state_valid = true;
        portEXIT_CRITICAL(&s_mux);
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    TickType_t t_log = xTaskGetTickCount();

    for (;;) {
        const int n = uart_read_bytes(LINK_UART_NUM, buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n > 0) {
            Proto_Feed(&s_parser, buf, (uint16_t)n);   /* only this task calls Proto_Feed */
        }

#if LINK_DEBUG_LOG
        /* Once per second: show what the STM32 reports (helps first bring-up) */
        if ((TickType_t)(xTaskGetTickCount() - t_log) >= pdMS_TO_TICKS(1000)) {
            t_log = xTaskGetTickCount();
            StatePayload st;
            uint32_t age_ms;
            if (stm32_link_get_state(&st, &age_ms)) {
                ESP_LOGI(TAG, "STM32 ok | age=%u ms flags=%u | pos[rad] M1_iL M2_iR M3_L M4_R = %.3f %.3f %.3f %.3f",
                         (unsigned)age_ms, (unsigned)st.flags,
                         (double)st.pos[0], (double)st.pos[1], (double)st.pos[2], (double)st.pos[3]);
            } else {
                ESP_LOGW(TAG, "no data from STM32 yet - check the 2 UART wires (crossed), GND and 115200 baud");
            }
        }
#endif
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    uint8_t frame[PROTO_MAX_PAYLOAD + PROTO_OVERHEAD];
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        CmdPayload c;
        portENTER_CRITICAL(&s_mux);
        c = s_cmd;
        portEXIT_CRITICAL(&s_mux);

        const uint16_t n = Proto_Build(MSG_CMD, &c, (uint8_t)sizeof c, frame);
        uart_write_bytes(LINK_UART_NUM, (const char *)frame, n);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(LINK_CMD_PERIOD_MS));
    }
}

void stm32_link_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART_NUM, LINK_TX_GPIO, LINK_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    Proto_Init(&s_parser, on_frame);

    xTaskCreate(rx_task, "stm32_rx", 3072, NULL, 6, NULL);
    xTaskCreate(tx_task, "stm32_tx", 3072, NULL, 6, NULL);
    ESP_LOGI(TAG, "UART%d up, %d baud, TX=GPIO%d RX=GPIO%d",
             (int)LINK_UART_NUM, LINK_BAUD, LINK_TX_GPIO, LINK_RX_GPIO);
}

void stm32_link_set_command(const CmdPayload *cmd)
{
    portENTER_CRITICAL(&s_mux);
    s_cmd = *cmd;
    portEXIT_CRITICAL(&s_mux);
}

bool stm32_link_get_state(StatePayload *out, uint32_t *age_ms)
{
    bool ok;
    TickType_t tick;
    portENTER_CRITICAL(&s_mux);
    ok = s_state_valid;
    if (ok) {
        *out = s_state;
        tick = s_state_tick;
    }
    portEXIT_CRITICAL(&s_mux);

    if (ok) {
        *age_ms = (uint32_t)((xTaskGetTickCount() - tick) * portTICK_PERIOD_MS);
    }
    return ok;
}
