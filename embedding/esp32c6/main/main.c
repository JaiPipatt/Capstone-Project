#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>

#include <rmw_microros/rmw_microros.h>   /* ping / reconnect: needs the XRCE-DDS middleware (default) */

#include "kinematics.h"
#include "protocol.h"
#include "robot_config.h"
#include "robot_logic.h"
#include "stm32_link.h"

/* Return false from the calling function on error (used while creating ROS entities). */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d.\n", __LINE__, (int)temp_rc); return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); } }

/* Agent connection check (micro-ROS reconnection pattern) */
#define AGENT_PING_PERIOD_MS    1000    /* while connected: ping the agent this often   */
#define AGENT_PING_TIMEOUT_MS   100     /* one ping attempt                             */
#define AGENT_PING_ATTEMPTS     3       /* lost after this many failed pings in a row   */
#define AGENT_RETRY_PERIOD_MS   1000    /* while waiting: try to reach the agent        */

/* ---------------------------------------------------------------------- */
/*  Kinematic parameters (from robot_config.h)                             */
/* ---------------------------------------------------------------------- */
static const KinParams KIN = {
    .wheel_radius_m = WHEEL_RADIUS_M,
    .track_width_m  = TRACK_WIDTH_M,
    .k_tilt         = K_TILT,
    .k_pan          = K_PAN,
    .pan_min        = PAN_MIN_RAD,
    .pan_max        = PAN_MAX_RAD,
    .tilt_min       = TILT_MIN_RAD,
    .tilt_max       = TILT_MAX_RAD,
};

/* ---------------------------------------------------------------------- */
/*  ROS objects (all of them are used ONLY from the micro-ROS task)        */
/* ---------------------------------------------------------------------- */
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_head;
static rcl_publisher_t    pub_head_state;
static rcl_publisher_t    pub_vel_state;
static rclc_support_t     support;
static rcl_node_t         node;
static rcl_timer_t        timer;
static rclc_executor_t    executor;
static rcl_allocator_t    allocator;
static bool               s_support_ok;        /* support (session) created -> must be cleaned up */

static geometry_msgs__msg__Twist              msg_cmd_vel_in;
static std_msgs__msg__Float64MultiArray       msg_head_in;     /* needs memory for its arrays */
static geometry_msgs__msg__Twist              msg_vel_out;
static geometry_msgs__msg__Vector3            msg_head_out;

static UserCmd g_user;                         /* last commands from ROS */

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Commands -> IK -> 4 motor targets -> STM32 (resent by the link task at 50 Hz).
    * Motor order: [0]=M1_iL [1]=M2_iR [2]=M3_L [3]=M4_R */
static void update_stm32_command(void)
{
    CmdPayload c;
    robot_build_stm32_cmd(&KIN, &g_user, now_ms(), CMD_VEL_TIMEOUT_MS, &c);
    stm32_link_set_command(&c);
}

/* /<ns>/diff_drive_controller/cmd_vel_unstamped  (same topic as the simulation) */
static void cmd_vel_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    if (!isfinite(m->linear.x) || !isfinite(m->angular.z)) return;
    robot_user_set_vel(&g_user, (float)m->linear.x, (float)m->angular.z, now_ms());
    update_stm32_command();
}

/* /<ns>/head_position_controller/commands  data = [tilt, pan] rad (same as the simulation) */
static void head_cb(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *m = (const std_msgs__msg__Float64MultiArray *)msgin;
    if (m->data.size < 2 || m->data.data == NULL) return;
    if (!isfinite(m->data.data[0]) || !isfinite(m->data.data[1])) return;
    robot_user_set_head(&g_user, (float)m->data.data[0], (float)m->data.data[1]);
    update_stm32_command();
}

/* Every STATE_PUBLISH_PERIOD_MS: refresh the command (handles the /cmd_vel timeout)
 * and publish feedback computed with forward kinematics. */
static void timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    update_stm32_command();

    StatePayload st;
    uint32_t age_ms = 0;
    if (!stm32_link_get_state(&st, &age_ms) || age_ms > STATE_STALE_MS) return;

    float pan, tilt;
    kin_pantilt_fk(&KIN, st.pos[0], st.pos[1], &pan, &tilt);
    msg_head_out.x = tilt;                       /* x = tilt, y = pan (same order as [tilt, pan]) */
    msg_head_out.y = pan;
    msg_head_out.z = 0.0;
    RCSOFTCHECK(rcl_publish(&pub_head_state, &msg_head_out, NULL));

    if (kin_wheels_valid(&KIN)) {
        float v, w;
        kin_wheels_fk(&KIN, st.vel[2], st.vel[3], &v, &w);
        msg_vel_out.linear.x  = v;
        msg_vel_out.angular.z = w;
        RCSOFTCHECK(rcl_publish(&pub_vel_state, &msg_vel_out, NULL));
    }
}

/* ---------------------------------------------------------------------- */
/*  micro-ROS entities: created when the agent answers, destroyed when lost */
/* ---------------------------------------------------------------------- */
/* Ping the agent before a session exists (needs the agent address in init options). */
static bool agent_reachable(void)
{
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    bool ok = (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                                CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) == RMW_RET_OK)
           && (rmw_uros_ping_agent_options(AGENT_PING_TIMEOUT_MS, 1, rmw_options) == RMW_RET_OK);
    (void)rcl_init_options_fini(&init_options);
    return ok;
}

static bool create_entities(void)
{
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    (void)rcl_init_options_fini(&init_options);
    RCCHECK(rc);
    s_support_ok = true;

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, MODULE_NODE_NAME, MODULE_NAMESPACE, &support));

    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_CMD_VEL));
    RCCHECK(rclc_subscription_init_default(
        &sub_head, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), TOPIC_HEAD_CMD));
    RCCHECK(rclc_publisher_init_default(
        &pub_head_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), TOPIC_HEAD_STATE));
    RCCHECK(rclc_publisher_init_default(
        &pub_vel_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_VEL_STATE));

    timer = rcl_get_zero_initialized_timer();
#if UROS_TIMER_INIT_DEFAULT2
    RCCHECK(rclc_timer_init_default2(&timer, &support,
                                     RCL_MS_TO_NS(STATE_PUBLISH_PERIOD_MS), timer_cb, true));
#else
    RCCHECK(rclc_timer_init_default(&timer, &support,
                                    RCL_MS_TO_NS(STATE_PUBLISH_PERIOD_MS), timer_cb));
#endif

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel_in,
                                           &cmd_vel_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_head, &msg_head_in,
                                           &head_cb, ON_NEW_DATA));
    return true;
}

/* Safe to call after a partial create_entities(): fini on a zero-initialized
 * entity just returns an error, which we ignore. */
static void destroy_entities(void)
{
    if (!s_support_ok) return;                 /* session never came up: nothing to clean */
    s_support_ok = false;

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context != NULL) {
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);  /* agent is gone: do not wait */
    }

    RCSOFTCHECK(rcl_publisher_fini(&pub_head_state, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_vel_state, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_cmd_vel, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_head, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

/* ---------------------------------------------------------------------- */
/*  micro-ROS task: wait for agent -> connect -> run -> (lost) -> clean up  */
/* ---------------------------------------------------------------------- */
typedef enum { AGENT_WAITING, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } AgentState;

static void micro_ros_task(void *arg)
{
    (void)arg;
    allocator = rcl_get_default_allocator();

    /* Memory for the arrays inside Float64MultiArray (data[], layout.dim[]).
     * Allocated once; it survives reconnections. */
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 16;
    conf.max_ros2_type_sequence_capacity = 2;
    conf.max_basic_type_sequence_capacity = 4;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), &msg_head_in, conf)) {
        printf("could not allocate memory for the head command message. Aborting.\n");
        vTaskDelete(NULL);
    }

    AgentState state = AGENT_WAITING;
    TickType_t t_check = xTaskGetTickCount();

    for (;;) {
        switch (state) {
        case AGENT_WAITING:
            /* No ROS executor running: keep applying the /cmd_vel timeout ourselves so
             * the wheels go to 0 rad/s instead of repeating the last command forever. */
            update_stm32_command();
            if ((TickType_t)(xTaskGetTickCount() - t_check) >= pdMS_TO_TICKS(AGENT_RETRY_PERIOD_MS)) {
                t_check = xTaskGetTickCount();
                if (agent_reachable()) {
                    state = AGENT_AVAILABLE;
                } else {
                    printf("micro-ROS agent %s:%s not reachable, retrying... "
                           "(agent running? same Wi-Fi? UDP port open in the firewall? IP correct?)\n",
                           CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                printf("micro-ROS agent connected\n");
                state = AGENT_CONNECTED;
            } else {
                destroy_entities();
                state = AGENT_WAITING;
            }
            t_check = xTaskGetTickCount();
            break;

        case AGENT_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if ((TickType_t)(xTaskGetTickCount() - t_check) >= pdMS_TO_TICKS(AGENT_PING_PERIOD_MS)) {
                t_check = xTaskGetTickCount();
                if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
                    printf("micro-ROS agent lost, reconnecting...\n");
                    state = AGENT_DISCONNECTED;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            break;

        case AGENT_DISCONNECTED:
        default:
            destroy_entities();
            update_stm32_command();
            state = AGENT_WAITING;
            t_check = xTaskGetTickCount();
            break;
        }
    }
}

// void app_main(void)
// {
// #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
//     printf("=== Wi-Fi network (SSID): \"%s\"  | micro-ROS agent: %s:%s ===\n",
//            CONFIG_ESP_WIFI_SSID, CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
// #endif
// #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
//     ESP_ERROR_CHECK(uros_network_interface_initialize());
// #endif

//     if (!kin_wheels_valid(&KIN)) {
//         printf("WARNING: WHEEL_RADIUS_M / TRACK_WIDTH_M are 0 in robot_config.h -> wheel commands are ignored\n");
//     }

//     stm32_link_init();

//     xTaskCreate(micro_ros_task, "uros_task",
//                 CONFIG_MICRO_ROS_APP_STACK, NULL,
//                 CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
// }

// to check the motor id
void app_main(void)
{
    // 1. เปิดการเชื่อมต่อ UART กับ STM32
    stm32_link_init(); 

    CmdPayload c;
    int test_index = 0;

    while(1) {
        // 2. สั่งหยุดมอเตอร์ทุกตัวก่อน 2 วินาที (เพื่อความปลอดภัยและให้เห็นจังหวะหยุด)
        memset(&c, 0, sizeof(c));
        stm32_link_set_command(&c);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 3. สั่งหมุนมอเตอร์แค่ตัวเดียว (จ่ายไฟ 30% = 0.3f) เป็นเวลา 2 วินาที
        memset(&c, 0, sizeof(c));
        c.mode[test_index] = MODE_DUTY; 
        c.target[test_index] = 0.3f;    
        stm32_link_set_command(&c);
        
        // แสดงสถานะผ่านไฟ LED หรือแค่หน่วงเวลา (สำหรับรันด้วยแบตเตอรี่)
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 4. สลับไปทดสอบมอเตอร์ตัวถัดไป (0 -> 1 -> 2 -> 3 -> วนกลับ 0)
        test_index++;
        if (test_index > 3) test_index = 0;
    }
}