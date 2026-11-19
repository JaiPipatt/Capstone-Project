#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Motor index order (same on ESP32, STM32 and test_stm32.py):
 *   [0] M1_iL = inner left  gear   (TIM4 enc, TIM3_CH3 PWM, PA4/PA5)
 *   [1] M2_iR = inner right gear   (TIM2 enc, TIM3_CH2 PWM, PB14/PB15)
 *   [2] M3_L  = outer left  wheel  (TIM1 enc, TIM3_CH1 PWM, PB12/PB13)
 *   [3] M4_R  = outer right wheel  (TIM5 enc, TIM3_CH4 PWM, PB4/PB5)
 * NOTE: the CubeMX pin labels in the .ioc / main.h still use the OLD names
 * (TB_1_B_..._M2_R, TB_2_B_..._M4_IR, ENC_M4_IR_...). They are the same pins;
 * only the physical role differs. Rename them in CubeMX if you regenerate.
 */
typedef enum {
    MOTOR_M1_IL = 0,
    MOTOR_M2_IR,
    MOTOR_M3_L,
    MOTOR_M4_R,
    MOTOR_COUNT
} MotorId;

/* Start PWM + encoders, put all bridges in coast, then raise STBY.
 * Calls Error_Handler() if an ENC_CPR_Mx in config.h is still 0. */
void  Motor_Init(void);

/* duty in -1..+1 (clamped to +-DUTY_LIMIT). 0 = short brake. */
void  Motor_SetDuty(MotorId id, float duty);
void  Motor_Coast(MotorId id);   /* IN1 = IN2 = L : bridge off, shaft free  */
void  Motor_Brake(MotorId id);   /* IN1 = IN2 = H : short brake             */

/* Call once per control tick, BEFORE using GetPos/GetVel. */
void  Motor_UpdateEncoder(MotorId id, float dt);

float Motor_GetPos(MotorId id);      /* rad, since power-up or last ZeroPos */
float Motor_GetVel(MotorId id);      /* rad/s, low-pass filtered            */
void  Motor_ZeroPos(MotorId id);

void  Motor_DriverEnable(bool enable);   /* TB6612 STBY pin */

#endif /* MOTOR_H */
