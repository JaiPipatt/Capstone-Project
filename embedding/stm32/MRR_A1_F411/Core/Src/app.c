#include "app.h"
#include "config.h"
#include "motor.h"
#include "pid.h"
#include "protocol.h"

#include "main.h"      /* LED_Pin, LED_GPIO_Port, Error_Handler */
#include "tim.h"       /* htim10                                */
#include "usart.h"     /* huart2                                */
#include "usb_device.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS, USBD_CDC_HandleTypeDef */

#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------- */
/*  Shared state (written by UART parser at IRQ prio 2, read by TIM10 ISR  */
/*  at IRQ prio 1 -> parser must use a critical section when writing)      */
/* ---------------------------------------------------------------------- */
static CtrlMode          g_mode[MOTOR_COUNT];
static float             g_target[MOTOR_COUNT];
static volatile uint32_t g_last_cmd_ms;
static volatile uint8_t  g_watchdog_tripped;

/* Control-loop private state (only touched inside TIM10 ISR) */
static PidCtrl  g_vel_pid[MOTOR_COUNT];
static PidCtrl  g_pos_pid[MOTOR_COUNT];
static float    g_vel_ref[MOTOR_COUNT];
static CtrlMode g_prev_mode[MOTOR_COUNT];
static uint8_t  g_pos_div_cnt;

extern USBD_HandleTypeDef hUsbDeviceFS;   /* defined by CubeMX in usb_device.c */

/* One frame parser per input channel (ESP32 UART, PC over USB) */
static ProtoParser g_parser_uart;
static ProtoParser g_parser_usb;
static uint8_t     g_usb_tx_buf[PROTO_MAX_PAYLOAD + PROTO_OVERHEAD];

/* UART */
#define RX_BUF_SIZE 256U
static uint8_t           g_rx_buf[RX_BUF_SIZE];
static uint16_t          g_rx_last_pos;
static volatile uint8_t  g_rx_restart;
static uint8_t           g_tx_buf[PROTO_MAX_PAYLOAD + PROTO_OVERHEAD];

/* Main-loop timers */
static uint32_t g_t_tele;
static uint32_t g_t_led;

/* ---------------------------------------------------------------------- */
/*  1 kHz control tick  (runs inside TIM10 update interrupt)               */
/*  Rules: no HAL_Delay, no blocking HAL_UART_Transmit, no printf here.    */
/* ---------------------------------------------------------------------- */
static void control_tick(void)
{
    const uint32_t now = HAL_GetTick();

    /* --- command watchdog: no valid CMD within CMD_TIMEOUT_MS -> STOP --- */
    if ((uint32_t)(now - g_last_cmd_ms) > CMD_TIMEOUT_MS) {
        g_watchdog_tripped = 1U;
        for (int i = 0; i < MOTOR_COUNT; i++) g_mode[i] = MODE_STOP;
    }

    int run_pos = 0;
    if (++g_pos_div_cnt >= POS_LOOP_DIV) {
        g_pos_div_cnt = 0U;
        run_pos = 1;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const MotorId id = (MotorId)i;

        Motor_UpdateEncoder(id, CTRL_DT_S);

        const CtrlMode mode = g_mode[i];
        if (mode != g_prev_mode[i]) {              /* mode changed -> clean start */
            Pid_Reset(&g_vel_pid[i]);
            Pid_Reset(&g_pos_pid[i]);
            g_vel_ref[i] = 0.0f;
            g_prev_mode[i] = mode;
        }

        switch (mode) {
        case MODE_VELOCITY:
            g_vel_ref[i] = g_target[i];
            break;

        case MODE_POSITION:
            if (run_pos) {
                g_vel_ref[i] = Pid_Update(&g_pos_pid[i], g_target[i],
                                          Motor_GetPos(id),
                                          CTRL_DT_S * (float)POS_LOOP_DIV);
            }
            break;

        case MODE_DUTY:
            Motor_SetDuty(id, g_target[i]);
            continue;                              /* skip the velocity PID */

        case MODE_STOP:
        default:
            Motor_Coast(id);
            continue;
        }

        /* Inner loop: velocity PI + optional feed-forward -> duty */
        const float u = VEL_KFF * g_vel_ref[i]
                      + Pid_Update(&g_vel_pid[i], g_vel_ref[i],
                                   Motor_GetVel(id), CTRL_DT_S);
        Motor_SetDuty(id, u);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        control_tick();
    }
}

/* ---------------------------------------------------------------------- */
/*  Incoming frames                                                        */
/* ---------------------------------------------------------------------- */
static void on_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
    if (type == MSG_CMD && len == sizeof(CmdPayload)) {
        CmdPayload c;
        memcpy(&c, p, sizeof c);                   /* memcpy: avoids unaligned access */

        CtrlMode m[MOTOR_COUNT];
        float    t[MOTOR_COUNT];
        for (int i = 0; i < MOTOR_COUNT; i++) {
            m[i] = (c.mode[i] <= (uint8_t)MODE_DUTY) ? (CtrlMode)c.mode[i] : MODE_STOP;
            t[i] = c.target[i];
            if (!isfinite(t[i])) { m[i] = MODE_STOP; t[i] = 0.0f; }   /* reject NaN/Inf */
        }

        __disable_irq();                           /* short critical section */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            g_mode[i]   = m[i];
            g_target[i] = t[i];
        }
        g_last_cmd_ms      = HAL_GetTick();
        g_watchdog_tripped = 0U;
        __enable_irq();

    } else if (type == MSG_ZERO && len == sizeof(ZeroPayload)) {
        ZeroPayload z;
        memcpy(&z, p, sizeof z);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (z.mask & (1U << i)) Motor_ZeroPos((MotorId)i);
        }
    }
    /* unknown type or wrong length: ignore */
}

/* ---------------------------------------------------------------------- */
/*  UART2 RX: circular DMA + IDLE line. Called on IDLE, half and full.     */
/*  Size = how many bytes the DMA has written since the start of g_rx_buf. */
/* ---------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) return;
    if (Size > RX_BUF_SIZE) return;

    if (Size > g_rx_last_pos) {
        Proto_Feed(&g_parser_uart, &g_rx_buf[g_rx_last_pos], (uint16_t)(Size - g_rx_last_pos));
    } else if (Size < g_rx_last_pos) {             /* DMA wrapped around */
        Proto_Feed(&g_parser_uart, &g_rx_buf[g_rx_last_pos], (uint16_t)(RX_BUF_SIZE - g_rx_last_pos));
        Proto_Feed(&g_parser_uart, &g_rx_buf[0], Size);
    }
    g_rx_last_pos = (Size == RX_BUF_SIZE) ? 0U : Size;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        g_rx_restart = 1U;                         /* restart from main loop (not in ISR) */
    }
}

static void uart_rx_start(void)
{
    g_rx_last_pos = 0U;
    Proto_Reset(&g_parser_uart);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_rx_buf, RX_BUF_SIZE);
}

/* ---------------------------------------------------------------------- */
/*  Telemetry -> ESP32 (50 Hz, non-blocking DMA)                           */
/* ---------------------------------------------------------------------- */
static void send_state(void)
{
    StatePayload s;
    s.stamp_ms = HAL_GetTick();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        s.pos[i] = Motor_GetPos((MotorId)i);
        s.vel[i] = Motor_GetVel((MotorId)i);
    }
    s.flags = g_watchdog_tripped ? 0x01U : 0x00U;

    /* 1) UART to the ESP32: build the frame only when the previous DMA is done */
    if (huart2.gState == HAL_UART_STATE_READY) {
        const uint16_t n = Proto_Build(MSG_STATE, &s, (uint8_t)sizeof s, g_tx_buf);
        HAL_UART_Transmit_DMA(&huart2, g_tx_buf, n);
    }

    /* 2) Same frame to the PC over USB-C (only when a PC has enumerated the device) */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hcdc != NULL && hcdc->TxState == 0U) {
        const uint16_t n = Proto_Build(MSG_STATE, &s, (uint8_t)sizeof s, g_usb_tx_buf);
        (void)CDC_Transmit_FS(g_usb_tx_buf, n);
    }
}

/* ---------------------------------------------------------------------- */
/*  Public API                                                             */
/* ---------------------------------------------------------------------- */
void App_Init(void)
{
    Motor_Init();                                   /* may call Error_Handler() */

    for (int i = 0; i < MOTOR_COUNT; i++) {
        /* The PID output limit must equal the REAL power limit (DUTY_LIMIT). If the PID thought it could
         * output +-1.0 while Motor_SetDuty() clamps to DUTY_LIMIT, the integral would keep growing during
         * saturation (windup) and the wheel would overshoot / react slowly afterwards. */
        Pid_Init(&g_vel_pid[i], VEL_KP, VEL_KI, VEL_KD, -DUTY_LIMIT, DUTY_LIMIT);
        Pid_Init(&g_pos_pid[i], POS_KP, POS_KI, POS_KD,
                 -POS_MAX_VEL_RAD_S, POS_MAX_VEL_RAD_S);
        g_mode[i]      = MODE_STOP;
        g_prev_mode[i] = MODE_STOP;
        g_target[i]    = 0.0f;
        g_vel_ref[i]   = 0.0f;
    }
    g_watchdog_tripped = 1U;                        /* "no command yet" */
    g_last_cmd_ms      = HAL_GetTick();

    Proto_Init(&g_parser_uart, on_frame);
    Proto_Init(&g_parser_usb,  on_frame);
    uart_rx_start();

    g_t_tele = HAL_GetTick();
    g_t_led  = g_t_tele;

    HAL_TIM_Base_Start_IT(&htim10);                 /* start the 1 kHz loop LAST */
}

void App_MainLoop(void)
{
    const uint32_t now = HAL_GetTick();

    if (g_rx_restart) {
        g_rx_restart = 0U;
        HAL_UART_AbortReceive(&huart2);
        uart_rx_start();
    }

    if ((uint32_t)(now - g_t_tele) >= TELEMETRY_PERIOD_MS) {
        g_t_tele = now;
        send_state();
    }

    /* LED: slow blink = OK, medium blink (250 ms) = no command received */
    const uint32_t blink_ms = g_watchdog_tripped ? 250U : 500U;
    if ((uint32_t)(now - g_t_led) >= blink_ms) {
        g_t_led = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

/* Called from the USB interrupt (CDC_Receive_FS) */
void App_UsbRx(const uint8_t *data, uint32_t len)
{
    if (len > 0xFFFFU) len = 0xFFFFU;
    Proto_Feed(&g_parser_usb, data, (uint16_t)len);
}
