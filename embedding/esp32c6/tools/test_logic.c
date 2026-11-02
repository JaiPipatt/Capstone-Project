/* Host test (runs on your PC):
 *   gcc -std=gnu11 -Wall -Wextra -I../main test_logic.c ../main/robot_logic.c ../main/kinematics.c ../main/protocol.c -lm -o test_logic && ./test_logic
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "robot_logic.h"

static int fails = 0;
#define CHECK(name, cond) do { if (cond) printf("PASS  %s\n", name); else { printf("FAIL  %s\n", name); fails++; } } while (0)
static int near(float a, float b) { return fabsf(a - b) < 1e-4f; }

int main(void)
{
    KinParams kin = { 0.050f, 0.09665f, 1.0f, 48.0f / 46.0f, -0.5f, 0.5f, -0.5f, 0.5f };
    UserCmd u; memset(&u, 0, sizeof u);
    CmdPayload o;

    robot_build_stm32_cmd(&kin, &u, 0, 500, &o);
    CHECK("before any command: all STOP", !o.mode[0] && !o.mode[1] && !o.mode[2] && !o.mode[3]);

    robot_user_set_vel(&u, 0.049f, 0.0f, 1000);
    robot_build_stm32_cmd(&kin, &u, 1100, 500, &o);
    CHECK("0.049 m/s -> 0.98 rad/s on both wheels", o.mode[0] == MODE_VELOCITY && o.mode[3] == MODE_VELOCITY && near(o.target[0], 0.98f) && near(o.target[3], 0.98f));
    CHECK("inner gears still STOP", !o.mode[1] && !o.mode[2]);

    robot_build_stm32_cmd(&kin, &u, 1600, 500, &o);
    CHECK("silent > 500 ms -> wheels hold 0 rad/s", o.mode[0] == MODE_VELOCITY && near(o.target[0], 0.0f));

    robot_user_set_vel(&u, 0.03f, 0.5f, 2000);
    robot_build_stm32_cmd(&kin, &u, 2010, 500, &o);
    CHECK("turning: left slower than right", o.target[0] < o.target[3] && near(o.target[0], 0.1168f) && near(o.target[3], 1.0832f));

    robot_user_set_head(&u, 0.2f, 0.3f);
    robot_build_stm32_cmd(&kin, &u, 2020, 500, &o);
    CHECK("head tilt 0.2 pan 0.3 -> spur 0.4875 / -0.0875", o.mode[2] == MODE_POSITION && o.mode[1] == MODE_POSITION && near(o.target[2], 0.4875f) && near(o.target[1], -0.0875f));

    robot_user_set_head(&u, 5.0f, 5.0f);
    robot_build_stm32_cmd(&kin, &u, 2030, 500, &o);
    CHECK("limits +-0.5 rad", near(o.target[2] + o.target[1], 1.0f));

    printf(fails ? "\n%d FAILED\n" : "\nALL OK\n", fails);
    return fails != 0;
}
