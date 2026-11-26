#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <stdbool.h>

/* One command from the PC, whatever the transport (UDP text line or micro-ROS topics). */
typedef struct {
    float v, w;         /* m/s, rad/s  (Twist linear.x, angular.z; w counter-clockwise +) */
    float tilt, pan;    /* rad         (head command [tilt, pan])                         */
    bool  has_head;     /* true if tilt/pan are valid                                     */
} RobotCmd;

#endif /* ROBOT_CMD_H */
