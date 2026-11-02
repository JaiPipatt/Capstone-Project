#include "robot_logic.h"

#include <string.h>

void robot_user_set_vel(UserCmd *u, float v, float w, uint32_t now_ms)
{
    u->v        = v;
    u->w        = w;
    u->t_vel_ms = now_ms;
    u->vel_seen = true;
}

void robot_user_set_head(UserCmd *u, float tilt, float pan)
{
    u->tilt      = tilt;
    u->pan       = pan;
    u->head_seen = true;
}

void robot_user_update(UserCmd *u, const RobotCmd *c, uint32_t now_ms)
{
    robot_user_set_vel(u, c->v, c->w, now_ms);
    if (c->has_head) {
        robot_user_set_head(u, c->tilt, c->pan);
    }
}

void robot_build_stm32_cmd(const KinParams *kin, const UserCmd *u, uint32_t now_ms,
                           uint32_t vel_timeout_ms, CmdPayload *out)
{
    memset(out, 0, sizeof *out);                    /* all four motors MODE_STOP */

    /* Outer wheels (M1 left, M4 right): velocity control.
     * After the first command, a silent PC means "hold 0 rad/s" (not "keep going"). */
    if (kin_wheels_valid(kin) && u->vel_seen) {
        float v = 0.0f, w = 0.0f;
        if ((uint32_t)(now_ms - u->t_vel_ms) < vel_timeout_ms) {
            v = u->v;
            w = u->w;
        }
        float wl, wr;
        kin_wheels_ik(kin, v, w, &wl, &wr);
        out->mode[0] = MODE_VELOCITY;  out->target[0] = wr;  /* M1_R */
        out->mode[1] = MODE_VELOCITY;  out->target[1] = wl;  /* M2_L */


    }

    /* Inner gears (M3, M2): position control of the two spur gears. */
    if (u->head_seen) {
        float q3, q4;
        kin_pantilt_ik(kin, u->pan, u->tilt, &q3, &q4);
        out->mode[2] = MODE_POSITION;  out->target[2] = q3;  /* M3_IR */
        out->mode[3] = MODE_POSITION;  out->target[3] = q4;  /* M4_IL */


    }
}
