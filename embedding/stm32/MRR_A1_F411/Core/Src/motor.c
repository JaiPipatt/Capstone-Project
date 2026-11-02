#include "motor.h"
#include "config.h"
#include "main.h"      /* pin label macros: TB_1_A_IN1_M1_L_Pin, ... */
#include "tim.h"       /* htim1, htim2, htim3, htim4, htim5          */
#include <math.h>

typedef struct {
    /* --- hardware (constant) --- */
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
    TIM_HandleTypeDef *pwm_htim;
    uint32_t           pwm_ch;
    TIM_HandleTypeDef *enc_htim;
    uint8_t            enc_is_32bit;
    int8_t             dir_sign;
    int8_t             enc_sign;
    float              counts_per_rev;
    /* --- state (changes at run time) --- */
    uint32_t           prev_cnt;
    int32_t            pos_counts;
    float              vel_rad_s;
} MotorHw;

static MotorHw g_m[MOTOR_COUNT] = {
    [MOTOR_M1_L] = {
        TB_1_A_IN1_M1_L_GPIO_Port, TB_1_A_IN1_M1_L_Pin,
        TB_1_A_IN2_M1_L_GPIO_Port, TB_1_A_IN2_M1_L_Pin,
        &htim3, TIM_CHANNEL_1,
        &htim1, 0U,
        MOTOR_DIR_SIGN_M1, ENC_SIGN_M1, ENC_CPR_M1, 0U, 0, 0.0f },
    [MOTOR_M2_IR] = {
        TB_1_B_IN1_M2_R_GPIO_Port, TB_1_B_IN1_M2_R_Pin,
        TB_1_B_IN2_M2_R_GPIO_Port, TB_1_B_IN2_M2_R_Pin,
        &htim3, TIM_CHANNEL_2,
        &htim2, 1U,
        MOTOR_DIR_SIGN_M2, ENC_SIGN_M2, ENC_CPR_M2, 0U, 0, 0.0f },
    [MOTOR_M3_IL] = {
        TB_2_A_IN1_M3_IL_GPIO_Port, TB_2_A_IN1_M3_IL_Pin,
        TB_2_A_IN2_M3_IL_GPIO_Port, TB_2_A_IN2_M3_IL_Pin,
        &htim3, TIM_CHANNEL_3,
        &htim4, 0U,
        MOTOR_DIR_SIGN_M3, ENC_SIGN_M3, ENC_CPR_M3, 0U, 0, 0.0f },
    [MOTOR_M4_R] = {
        TB_2_B_IN1_M4_IR_GPIO_Port, TB_2_B_IN1_M4_IR_Pin,
        TB_2_B_IN2_M4_IR_GPIO_Port, TB_2_B_IN2_M4_IR_Pin,
        &htim3, TIM_CHANNEL_4,
        &htim5, 1U,
        MOTOR_DIR_SIGN_M4, ENC_SIGN_M4, ENC_CPR_M4, 0U, 0, 0.0f },
};

static inline void set_in(const MotorHw *m, GPIO_PinState in1, GPIO_PinState in2)
{
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, in1);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, in2);
}

#if STBY_ALSO_DRIVE_PB10
/* PB10 is not configured by CubeMX in this project, so set it up here. */
static void stby_alt_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_10;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
#endif

void Motor_DriverEnable(bool enable)
{
    const GPIO_PinState level = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(TB_STBY_BOTH_GPIO_Port, TB_STBY_BOTH_Pin, level);
#if STBY_ALSO_DRIVE_PB10
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, level);
#endif
}

void Motor_Init(void)
{
#if STBY_ALSO_DRIVE_PB10
    stby_alt_init();
#endif
    Motor_DriverEnable(false);               /* driver in standby while we set up */

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorHw *m = &g_m[i];

        if (m->counts_per_rev <= 0.0f) {
            /* config.h not filled in (MOTOR_GEARBOX_RATIO is still 0).
             * Motors stay OFF; LED blinks very fast (40 ms) forever. */
            for (;;) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_Delay(40);
            }
        }

        set_in(m, GPIO_PIN_RESET, GPIO_PIN_RESET);              /* coast */
        __HAL_TIM_SET_COMPARE(m->pwm_htim, m->pwm_ch, 0U);
        HAL_TIM_PWM_Start(m->pwm_htim, m->pwm_ch);

        __HAL_TIM_SET_COUNTER(m->enc_htim, 0U);
        HAL_TIM_Encoder_Start(m->enc_htim, TIM_CHANNEL_ALL);

        m->prev_cnt   = 0U;
        m->pos_counts = 0;
        m->vel_rad_s  = 0.0f;
    }

    Motor_DriverEnable(true);
}

void Motor_Coast(MotorId id)
{
    MotorHw *m = &g_m[id];
    __HAL_TIM_SET_COMPARE(m->pwm_htim, m->pwm_ch, 0U);
    set_in(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void Motor_Brake(MotorId id)
{
    MotorHw *m = &g_m[id];
    __HAL_TIM_SET_COMPARE(m->pwm_htim, m->pwm_ch, 0U);
    set_in(m, GPIO_PIN_SET, GPIO_PIN_SET);
}

void Motor_SetDuty(MotorId id, float duty)
{
    MotorHw *m = &g_m[id];

    if (duty >  DUTY_LIMIT) duty =  DUTY_LIMIT;
    if (duty < -DUTY_LIMIT) duty = -DUTY_LIMIT;
    duty *= (float)m->dir_sign;

    if (duty > 0.0f) {
        set_in(m, GPIO_PIN_SET, GPIO_PIN_RESET);     /* one direction  */
    } else if (duty < 0.0f) {
        set_in(m, GPIO_PIN_RESET, GPIO_PIN_SET);     /* other direction */
    } else {
        set_in(m, GPIO_PIN_SET, GPIO_PIN_SET);       /* zero -> short brake */
    }

    /* CCR in 0 .. ARR+1  (ARR+1 = always high = 100 %) */
    uint32_t ccr = (uint32_t)(fabsf(duty) * (float)(PWM_ARR + 1U));
    if (ccr > (PWM_ARR + 1U)) ccr = PWM_ARR + 1U;
    __HAL_TIM_SET_COMPARE(m->pwm_htim, m->pwm_ch, ccr);
}

void Motor_UpdateEncoder(MotorId id, float dt)
{
    MotorHw *m = &g_m[id];

    const uint32_t cnt = __HAL_TIM_GET_COUNTER(m->enc_htim);
    int32_t delta;

    if (m->enc_is_32bit) {
        delta = (int32_t)(cnt - m->prev_cnt);                  /* wraps correctly */
    } else {
        delta = (int32_t)(int16_t)((uint16_t)cnt - (uint16_t)m->prev_cnt);
    }
    m->prev_cnt = cnt;

    delta *= (int32_t)m->enc_sign;
    m->pos_counts += delta;

    const float raw = ((float)delta / dt) * (TWO_PI_F / m->counts_per_rev);
    m->vel_rad_s += VEL_LPF_ALPHA * (raw - m->vel_rad_s);
}

float Motor_GetPos(MotorId id)
{
    const MotorHw *m = &g_m[id];
    return (float)m->pos_counts * (TWO_PI_F / m->counts_per_rev);
}

float Motor_GetVel(MotorId id)
{
    return g_m[id].vel_rad_s;
}

void Motor_ZeroPos(MotorId id)
{
    g_m[id].pos_counts = 0;
}
