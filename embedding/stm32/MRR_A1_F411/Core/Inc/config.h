#ifndef CONFIG_H
#define CONFIG_H

/* ======================================================================
 *  config.h  -  every number you may need to change lives here
 *  Values marked  [FROM .IOC]  were read from MRR_A1_F411.ioc
 *  Values marked  [TODO]       you MUST measure / read from datasheets
 *  Values marked  [TUNE]       are starting points only, tune on the robot
 * ====================================================================== */

#define TWO_PI_F              6.28318530718f

/* ---------- TB6612 standby (STBY) pin ---------------------------------------
 * CONFIRMED by the person who wired the board: STBY is on PC15 (CubeMX label
 * TB_STBY_BOTH). The KiCad schematic and the report still say PB10 - those two
 * documents are out of date and should be corrected.
 * Debug option: if EVERYTHING else works but the motors never move, set this to 1
 * to also drive PB10 (harmless if PB10 is not connected), then re-check the wire. */
#define STBY_ALSO_DRIVE_PB10  0

/* ---------- Timing ---------------------------------------------------- */
/* TIM10: 96 MHz / (95+1) / (999+1) = 1 kHz                 [FROM .IOC]   */
#define CTRL_FREQ_HZ          1000.0f
#define CTRL_DT_S             (1.0f / CTRL_FREQ_HZ)
#define POS_LOOP_DIV          10U      /* outer position loop = 1 kHz / 10 = 100 Hz */
#define TELEMETRY_PERIOD_MS   20U      /* 50 Hz state -> ESP32                       */
#define CMD_TIMEOUT_MS        200U     /* command watchdog                           */

/* ---------- PWM ------------------------------------------------------- */
/* TIM3: 96 MHz / (0+1) / (4799+1) = 20 kHz                 [FROM .IOC]   */
#define PWM_ARR               4799U

/* Bring-up safety: never output more than this duty (0.0 .. 1.0).
 * Raise it only after the first tests work.                 [TUNE]        */
#define DUTY_LIMIT            1.0f

/* ---------- Encoders / gears (per motor) ------------------------------ */
/* From your answers:  encoder = 7 PPR (per channel).
 * Timer encoder mode TI12 counts every edge of A and B  ->  x4.            */
#define ENC_PPR               7.0f
#define ENC_DECODE_X          4.0f

/* Built-in gearbox of each N20 motor (between the motor shaft with the
 * encoder and the 9T pinion). You told me 298:1.
 * (If this is ever 0 the firmware refuses to start: LED blinks very fast.)   */
#define MOTOR_GEARBOX_RATIO   298.0f   /* N20 gearbox 298:1 (from you) */

/* Gear train (from the Capstone progress report, section 4.2.5):
 * ALL FOUR motors drive a 9T pinion that meshes with a 48T spur gear.
 *   M1, M4 (outer): 48T spur -> wheel
 *   M2, M3 (inner): 48T spur -> (the two inner spurs both mesh with the 46T crown gear;
 *                    the crown is handled by the pan/tilt kinematics on the ESP32,
 *                    NOT by this firmware)
 * So this firmware reports every motor as the ANGLE OF ITS 48T SPUR GEAR (rad).
 * G (MOTOR_GEARBOX_RATIO) = turns of the motor shaft (where the 7 PPR encoder is)
 * per ONE turn of the N20 output shaft that carries the 9T pinion.            */
#define PINION_TEETH          9.0f
#define SPUR_TEETH            48.0f

/* Counts per ONE revolution of the 48T spur gear:
 *   counts = 4 * PPR * G * (48 / 9)                                          */
#define ENC_CPR_SPUR    (ENC_DECODE_X * ENC_PPR * MOTOR_GEARBOX_RATIO * (SPUR_TEETH / PINION_TEETH))

#define ENC_CPR_M1  ENC_CPR_SPUR
#define ENC_CPR_M2  ENC_CPR_SPUR
#define ENC_CPR_M3  ENC_CPR_SPUR
#define ENC_CPR_M4  ENC_CPR_SPUR

/* +1 or -1. Flip until "positive duty" gives "positive encoder count". [TODO] */
// #define MOTOR_DIR_SIGN_M1     (-1)
// #define MOTOR_DIR_SIGN_M2     (+1) 
// #define MOTOR_DIR_SIGN_M3     (-1)
// #define MOTOR_DIR_SIGN_M4     (+1)
// #define ENC_SIGN_M1           (+1)
// #define ENC_SIGN_M2           (-1)
// #define ENC_SIGN_M3           (+1)
// #define ENC_SIGN_M4           (-1)

/* New M1_R gets old M4 values */
#define MOTOR_DIR_SIGN_M1  (+1)   /* old MOTOR_DIR_SIGN_M4 */
#define ENC_SIGN_M1        (-1)   /* old ENC_SIGN_M4 */

/* New M2_L gets old M1 values */
#define MOTOR_DIR_SIGN_M2  (-1)   /* old MOTOR_DIR_SIGN_M1 */
#define ENC_SIGN_M2        (+1)   /* old ENC_SIGN_M1 */

/* New M3_IR gets old M2 values */
#define MOTOR_DIR_SIGN_M3  (+1)   /* old MOTOR_DIR_SIGN_M2 */
#define ENC_SIGN_M3        (-1)   /* old ENC_SIGN_M2 */

/* New M4_IL gets old M3 values */
#define MOTOR_DIR_SIGN_M4  (-1)   /* old MOTOR_DIR_SIGN_M3 */
#define ENC_SIGN_M4        (+1)   /* old ENC_SIGN_M3 */

/* Low-pass on measured speed: v += alpha * (v_raw - v).
 * Smaller = smoother but slower.                                       [TUNE] */
#define VEL_LPF_ALPHA         0.10f

/* ---------- PID starting points                                     [TUNE] ----------
 * Chosen by SIMULATING this exact firmware against a motor model built from your
 * report (6 V, ~0.98 rad/s of the 48T spur gear at 100 % duty) and tried on 7 different
 * motor models (slow/fast, weak/strong, with and without a dead zone).
 * They are a SAFE START, not final values: tune on the real robot.              */
/* Velocity loop: input error in rad/s, output = duty (-1..+1)
 *   KFF = "guess the duty from the target speed": 0.8 duty per rad/s (~1/0.98 reduced
 *   a bit so a stronger-than-expected motor does not overshoot).                */
#define VEL_KP                0.30f
#define VEL_KI                1.50f
#define VEL_KD                0.00f
#define VEL_KFF               0.80f
/* Position loop (outer, 100 Hz): input error in rad, output = velocity reference (rad/s).
 * Kept LOW on purpose: the speed signal is grainy (1 encoder count in 1 ms = 0.14 rad/s). */
#define POS_KP                0.80f
#define POS_KI                0.00f
#define POS_KD                0.00f
#define POS_MAX_VEL_RAD_S     0.90f

#endif /* CONFIG_H */
