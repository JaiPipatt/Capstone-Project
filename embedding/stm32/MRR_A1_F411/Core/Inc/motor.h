#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Names follow the pin labels in the .ioc:
 *   M1_L  = outer left  wheel   (TIM1 enc, TIM3_CH1 PWM, PB12/PB13)
 *   M2_R  = outer right wheel   (TIM2 enc, TIM3_CH2 PWM, PB14/PB15)
 *   M3_IL = inner left  gear    (TIM4 enc, TIM3_CH3 PWM, PA4/PA5)
 *   M4_IR = inner right gear    (TIM5 enc, TIM3_CH4 PWM, PB4/PB5)
 */
typedef enum {
    MOTOR_M1_L = 0,
    MOTOR_M2_R,
    MOTOR_M3_IL,
    MOTOR_M4_IR,
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
