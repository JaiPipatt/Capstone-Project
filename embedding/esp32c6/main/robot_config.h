#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ======================================================================
 *  robot_config.h  -  everything you may need to change for ONE module
 *  [TODO] = you must set / measure it
 * ====================================================================== */

/* ---- ROS 2 names (same topics as the simulation: ros2_control + controllers.yaml) ---- */
#define MODULE_NAMESPACE        "/module_1"           /* one namespace per module: /module_1, /module_2, ... */
#define MODULE_NODE_NAME        "esp32_module"
#define TOPIC_CMD_VEL           "diff_drive_controller/cmd_vel_unstamped"   /* geometry_msgs/Twist          */
#define TOPIC_HEAD_CMD          "head_position_controller/commands"         /* std_msgs/Float64MultiArray [tilt, pan] rad */
#define TOPIC_HEAD_STATE        "esp32/head_state"    /* geometry_msgs/Vector3: x = tilt, y = pan [rad]  (feedback) */
#define TOPIC_VEL_STATE         "esp32/vel_state"     /* geometry_msgs/Twist: measured v, w            (feedback) */

/* micro-ROS component / ROS 2 distro. Your PC uses ROS 2 Humble (report, section 4.6):
 *   0 = Humble  (rclc_timer_init_default)   <- component version ^22
 *   1 = Jazzy   (rclc_timer_init_default2)  <- component version ^24           */
#define UROS_TIMER_INIT_DEFAULT2 0

/* ---- UART link to the STM32 (USART2 on the STM32, 115200 8N1) ----------- */
#define LINK_UART_NUM           UART_NUM_1
#define LINK_BAUD               115200
/* Taken from your schematic (ESP32-C6-Zero, U5):  GPIO4 = UART_E2S (ESP -> STM32),
 *                                                   GPIO5 = UART_S2E (STM32 -> ESP).
 *   ESP32 GPIO4 (TX)  ->  STM32 PA3 (USART2_RX, labelled ESP_TX)
 *   ESP32 GPIO5 (RX)  <-  STM32 PA2 (USART2_TX, labelled ESP_RX)
 *   GND <-> GND
 * NOTE: GPIO4 and GPIO5 are "strapping pins" of the ESP32-C6 (ESP-IDF docs): their
 * level at reset is read by the chip. UART normally works on them. If the ESP32
 * behaves strangely at boot only while the STM32 is powered/connected, suspect this
 * first and move to other pins.                                              */
#define LINK_TX_GPIO            4
#define LINK_RX_GPIO            5

/* 1 = print what the STM32 reports once per second (ESP-IDF monitor). */
#define LINK_DEBUG_LOG          1

/* ---- Timing ---------------------------------------------------------- */
#define LINK_CMD_PERIOD_MS      20      /* resend command to STM32 at 50 Hz (STM32 watchdog = 200 ms) */
#define CMD_VEL_TIMEOUT_MS      500     /* no /cmd_vel for this long -> wheels target 0 rad/s          */
#define STATE_PUBLISH_PERIOD_MS 50      /* publish feedback at 20 Hz                                    */
#define STATE_STALE_MS          500     /* do not publish if the STM32 has been silent this long        */

/* ---- Kinematic parameters -------------------------------------------- */
/* Differential drive (outer wheels). From your report, section 4.6.3
 * radius 50 mm; separation 99.9 mm (re-measured on the robot).             */
#define WHEEL_RADIUS_M          0.050f
#define TRACK_WIDTH_M           0.0999f

/* The installed outer-wheel direction is opposite to the ROS diff-drive
 * convention. Keep encoder signs unchanged and invert commanded wheel speed. */
#define DRIVE_WHEEL_COMMAND_SIGN (-1.0f)

/* Pan/tilt differential. q3, q4 = angle of the two inner 48T SPUR gears
 * (this is what the STM32 reports for M1 and M2).
 *   tilt = K_TILT * (q3 + q4) / 2
 *   pan  = K_PAN  * (q3 - q4) / 2
 * Same relation as in your xacro (my_robot.ros2_control.xacro):
 *   spur = tilt + pan * 46/48   ->   K_TILT = 1,  K_PAN = 48/46.
 * The SIGN depends on how the parts are assembled: command +0.3 rad and look;
 * if the face moves the wrong way, put a minus sign in front.     [TODO verify] */
#define K_TILT                  1.0f
#define K_PAN                   (48.0f / 46.0f)

/* Conservative bring-up limits for the face (rad). Widen after testing. [TODO] */
#define PAN_MIN_RAD            (-0.5f)
#define PAN_MAX_RAD             0.5f
#define TILT_MIN_RAD           (-0.5f)
#define TILT_MAX_RAD            0.5f

#endif /* ROBOT_CONFIG_H */
