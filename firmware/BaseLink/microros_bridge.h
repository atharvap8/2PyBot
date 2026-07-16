/*
 * ============================================================
 *  microros_bridge.h — micro-ROS Client (WiFi UDP transport)
 * ============================================================
 *  Turns the ESP32 into a micro-ROS node talking to the agent
 *  running on the onboard Radxa Cubie A7A:
 *
 *      docker compose up -d microros-agent   (udp4, port 8888)
 *
 *  Topics:
 *    sub  /cmd_vel       geometry_msgs/Twist   -> drive commands
 *    sub  /pybot/enable  std_msgs/Bool         -> motor arm/disarm
 *    pub  /pybot/pitch   std_msgs/Float32      (telemetry rate)
 *    pub  /pybot/state   std_msgs/Float32MultiArray
 *                        [pitch, setpoint, whlAngL, whlAngR,
 *                         speedL, speedR]
 *
 *  Design notes:
 *    - Writes the SAME volatile joy* globals as espnow_comm.h, so
 *      the main loop's drive logic is unchanged. Enable USE_MICROROS
 *      in config.h to switch input source from ESP-NOW to ROS.
 *    - Non-blocking: agent discovery/reconnect handled by a small
 *      state machine; the balance loop never waits on the network.
 *    - Requires the micro_ros_arduino library (humble branch).
 * ============================================================
 */

#ifndef MICROROS_BRIDGE_H
#define MICROROS_BRIDGE_H

#include <Arduino.h>
#include "config.h"

#if USE_MICROROS

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

// Shared input globals (defined in espnow_comm.h, reused here so the
// main loop's joystick handling works unmodified).
extern volatile float         joyForward;
extern volatile float         joySteering;
extern volatile uint8_t       joyEnable;
extern volatile unsigned long lastJoyPacketMs;

// ------------------------------------------------------------
//  State
// ------------------------------------------------------------
enum MicroRosState {
    MR_WAITING_AGENT,
    MR_CONNECTING,
    MR_CONNECTED,
    MR_DISCONNECTED
};

static MicroRosState mrState = MR_WAITING_AGENT;

static rcl_allocator_t   mrAllocator;
static rclc_support_t    mrSupport;
static rcl_node_t        mrNode;
static rclc_executor_t   mrExecutor;

static rcl_subscription_t subCmdVel;
static rcl_subscription_t subEnable;
static rcl_publisher_t    pubPitch;
static rcl_publisher_t    pubState;

static geometry_msgs__msg__Twist        msgCmdVel;
static std_msgs__msg__Bool              msgEnable;
static std_msgs__msg__Float32           msgPitch;
static std_msgs__msg__Float32MultiArray msgState;
static float mrStateBuffer[6];

static unsigned long mrLastPingMs = 0;

// ------------------------------------------------------------
//  Callbacks (executor context — keep them tiny)
// ------------------------------------------------------------
static void onCmdVel(const void *msgin) {
    const geometry_msgs__msg__Twist *m =
        (const geometry_msgs__msg__Twist *)msgin;

    // Map to the normalized joystick convention used by the main loop:
    // desired lean = -joyForward, so forward (+x) needs joyForward < 0.
    float fwd   = constrain((float)m->linear.x  / MR_MAX_LINEAR,  -1.0f, 1.0f);
    float steer = constrain((float)m->angular.z / MR_MAX_ANGULAR, -1.0f, 1.0f);

    joyForward      = -fwd;
    joySteering     = steer;
    lastJoyPacketMs = millis();
}

static void onEnable(const void *msgin) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    joyEnable       = m->data ? 1 : 0;
    lastJoyPacketMs = millis();
}

// ------------------------------------------------------------
//  Entity lifecycle
// ------------------------------------------------------------
static bool mrCreateEntities() {
    mrAllocator = rcl_get_default_allocator();

    if (rclc_support_init(&mrSupport, 0, NULL, &mrAllocator) != RCL_RET_OK)
        return false;

    if (rclc_node_init_default(&mrNode, "pybot_esp32", "", &mrSupport) != RCL_RET_OK)
        return false;

    rclc_subscription_init_default(
        &subCmdVel, &mrNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    rclc_subscription_init_default(
        &subEnable, &mrNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "pybot/enable");

    rclc_publisher_init_best_effort(
        &pubPitch, &mrNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pybot/pitch");

    rclc_publisher_init_best_effort(
        &pubState, &mrNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "pybot/state");

    // Static storage for the state array (no heap churn in the loop).
    msgState.data.data     = mrStateBuffer;
    msgState.data.size     = 6;
    msgState.data.capacity = 6;

    rclc_executor_init(&mrExecutor, &mrSupport.context, 2, &mrAllocator);
    rclc_executor_add_subscription(&mrExecutor, &subCmdVel, &msgCmdVel,
                                   &onCmdVel, ON_NEW_DATA);
    rclc_executor_add_subscription(&mrExecutor, &subEnable, &msgEnable,
                                   &onEnable, ON_NEW_DATA);
    return true;
}

static void mrDestroyEntities() {
    rcl_subscription_fini(&subCmdVel, &mrNode);
    rcl_subscription_fini(&subEnable, &mrNode);
    rcl_publisher_fini(&pubPitch, &mrNode);
    rcl_publisher_fini(&pubState, &mrNode);
    rclc_executor_fini(&mrExecutor);
    rcl_node_fini(&mrNode);
    rclc_support_fini(&mrSupport);
}

// ------------------------------------------------------------
//  Public API
// ------------------------------------------------------------

// Call once in setup(). Connects WiFi and points the UDP transport
// at the agent. Non-fatal on failure — the bridge keeps retrying.
inline void microros_begin() {
    Serial.printf("[uROS] WiFi connecting to %s\n", MR_WIFI_SSID);
    set_microros_wifi_transports(
        (char *)MR_WIFI_SSID, (char *)MR_WIFI_PASS,
        (char *)MR_AGENT_IP, MR_AGENT_PORT);
    mrState = MR_WAITING_AGENT;
    Serial.printf("[uROS] Transport ready — agent %s:%d\n",
                  MR_AGENT_IP, MR_AGENT_PORT);
}

// Call every loop tick. Cheap when idle; never blocks the PID.
inline void microros_spin() {
    switch (mrState) {
        case MR_WAITING_AGENT:
            // Ping at 2 Hz until the agent answers.
            if (millis() - mrLastPingMs > 500) {
                mrLastPingMs = millis();
                if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK)
                    mrState = MR_CONNECTING;
            }
            break;

        case MR_CONNECTING:
            if (mrCreateEntities()) {
                mrState = MR_CONNECTED;
                Serial.println("[uROS] Agent connected — entities up");
            } else {
                mrDestroyEntities();
                mrState = MR_WAITING_AGENT;
            }
            break;

        case MR_CONNECTED:
            // Time-boxed spin: handle pending callbacks, never wait.
            rclc_executor_spin_some(&mrExecutor, RCL_MS_TO_NS(1));

            // Watchdog ping at 1 Hz.
            if (millis() - mrLastPingMs > 1000) {
                mrLastPingMs = millis();
                if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
                    Serial.println("[uROS] Agent lost — motors safe-stopped");
                    joyForward = 0.0f;
                    joySteering = 0.0f;
                    joyEnable = 0;
                    mrState = MR_DISCONNECTED;
                }
            }
            break;

        case MR_DISCONNECTED:
            mrDestroyEntities();
            mrState = MR_WAITING_AGENT;
            break;
    }
}

// Call at telemetry rate (PLOT_DIVIDER). No-op unless connected.
inline void microros_publish(float pitch, float setpoint,
                             float whlAngL, float whlAngR,
                             int32_t speedL, int32_t speedR) {
    if (mrState != MR_CONNECTED) return;

    msgPitch.data = pitch;
    rcl_publish(&pubPitch, &msgPitch, NULL);

    mrStateBuffer[0] = pitch;
    mrStateBuffer[1] = setpoint;
    mrStateBuffer[2] = whlAngL;
    mrStateBuffer[3] = whlAngR;
    mrStateBuffer[4] = (float)speedL;
    mrStateBuffer[5] = (float)speedR;
    rcl_publish(&pubState, &msgState, NULL);
}

inline bool microros_connected() { return mrState == MR_CONNECTED; }

#else  // !USE_MICROROS — no-op stubs so BaseLink.ino compiles either way

inline void microros_begin() {}
inline void microros_spin() {}
inline void microros_publish(float, float, float, float, int32_t, int32_t) {}
inline bool microros_connected() { return false; }

#endif // USE_MICROROS
#endif // MICROROS_BRIDGE_H
