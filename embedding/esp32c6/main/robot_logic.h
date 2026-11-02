#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_cmd.h"
#include "kinematics.h"
#include "protocol.h"

/* What the PC last asked for. */
typedef struct {
    bool     vel_seen;          /* at least one wheel command received */
    float    v, w;              /* m/s, rad/s                          */
    uint32_t t_vel_ms;          /* time the last wheel command arrived */
    bool     head_seen;         /* at least one 4-number command       */
    float    tilt, pan;         /* rad                                 */
} UserCmd;

/* Store a freshly parsed command. */
void robot_user_update(UserCmd *u, const RobotCmd *c, uint32_t now_ms);

/* Same thing, one part at a time (used when wheels and head arrive on different topics). */
void robot_user_set_vel(UserCmd *u, float v, float w, uint32_t now_ms);
void robot_user_set_head(UserCmd *u, float tilt, float pan);

/* Inverse kinematics: user command -> mode + target for the 4 STM32 motors
 * [0]=M1_R [1]=M2_L [2]=M3_IR [3]=M4_IL.
 * Wheels: rad/s of the wheel. Inner gears: rad of the 48T spur gear. */
void robot_build_stm32_cmd(const KinParams *kin, const UserCmd *u, uint32_t now_ms,
                           uint32_t vel_timeout_ms, CmdPayload *out);

#endif /* ROBOT_LOGIC_H */
