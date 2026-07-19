/*
 * ============================================================
 *  BaseLinkSlave.ino — micro-ROS Hardware Slave
 * ============================================================
 *  EXPERIMENT (feature/ros-balancing): ALL control logic lives
 *  in ROS 2 on the Radxa. This firmware is a dumb, fast I/O
 *  bridge:
 *
 *    pub  /pybot/imu_state   std_msgs/Float32MultiArray @ 200 Hz
 *         [0] pitch (deg)       [1] pitch rate (deg/s)
 *         [2] yaw (deg)         [3] enc ticks L
 *         [4] enc ticks R       [5] loop dt (s)
 *    sub  /pybot/wheel_cmd    std_msgs/Float32MultiArray
 *         [0] speed L (steps/s) [1] speed R (steps/s)
 *    sub  /pybot/enable       std_msgs/Bool
 *
 *  Safety that CANNOT move off-chip:
 *    - command watchdog: wheel_cmd older than CMD_TIMEOUT_MS
 *      -> motors stop (a laggy network must never drive us)
 *    - hard tilt cutoff at MAX_TILT_ANGLE -> disable + notify
 *
 *  Transport: USB serial to the agent on the Radxa (dedicated;
 *  no debug prints on Serial — use the onboard LED for status).
 *
 *  Checkpoint to revert to on-chip balancing:
 *      git checkout checkpoint-esp32-balance
 * ============================================================
 */

#include "config.h"
#include "imu_sensor.h"
#include "stepper_control.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

// ------------------------------------------------------------
//  Slave-specific tunables
// ------------------------------------------------------------
#define CMD_TIMEOUT_MS   150   // stop motors if ROS goes quiet
#define IMU_PUB_HZ       200   // matches old on-chip loop rate

IMUSensor imu;

// ---- micro-ROS entities ----
static rcl_allocator_t   allocator;
static rclc_support_t    support;
static rcl_node_t        node;
static rclc_executor_t   executor;

static rcl_publisher_t    pubImu;
static rcl_subscription_t subWheel;
static rcl_subscription_t subEnable;

static std_msgs__msg__Float32MultiArray msgImu;
static std_msgs__msg__Float32MultiArray msgWheel;
static std_msgs__msg__Bool              msgEnable;
static float imuBuf[6];
static float wheelBuf[2];

// ---- state ----
enum SlaveState { WAITING_AGENT, CONNECTING, RUNNING };
static SlaveState slaveState = WAITING_AGENT;

static volatile int32_t cmdL = 0, cmdR = 0;
static volatile unsigned long lastCmdMs = 0;
static bool motorsEnabled = false;

static unsigned long lastLoopUs = 0;
static unsigned long lastPubUs  = 0;
static unsigned long lastPingMs = 0;

// ------------------------------------------------------------
//  Callbacks
// ------------------------------------------------------------
static void onWheelCmd(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *m =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (m->data.size >= 2) {
        cmdL = (int32_t)m->data.data[0];
        cmdR = (int32_t)m->data.data[1];
        lastCmdMs = millis();
    }
}

static void onEnable(const void *msgin) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    if (m->data && !motorsEnabled) {
        steppers.enable();
        motorsEnabled = true;
    } else if (!m->data && motorsEnabled) {
        steppers.setSpeed(0);
        steppers.disable();
        motorsEnabled = false;
    }
}

// ------------------------------------------------------------
//  Entity lifecycle
// ------------------------------------------------------------
static bool createEntities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pybot_esp32_slave", "", &support) != RCL_RET_OK) return false;

    rclc_publisher_init_best_effort(
        &pubImu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "pybot/imu_state");

    rclc_subscription_init_best_effort(
        &subWheel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "pybot/wheel_cmd");

    rclc_subscription_init_default(
        &subEnable, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "pybot/enable");

    msgImu.data.data = imuBuf;
    msgImu.data.size = 6;
    msgImu.data.capacity = 6;

    // Incoming wheel command needs backing storage too.
    msgWheel.data.data = wheelBuf;
    msgWheel.data.size = 0;
    msgWheel.data.capacity = 2;

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subWheel, &msgWheel, &onWheelCmd, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subEnable, &msgEnable, &onEnable, ON_NEW_DATA);
    return true;
}

static void destroyEntities() {
    rcl_publisher_fini(&pubImu, &node);
    rcl_subscription_fini(&subWheel, &node);
    rcl_subscription_fini(&subEnable, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ------------------------------------------------------------
//  SETUP
// ------------------------------------------------------------
void setup() {
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);

    // Serial owned by micro-ROS — no text output anywhere.
    set_microros_transports();

    if (!imu.begin()) {
        // IMU dead: fast blink forever, never enable motors.
        while (true) {
            digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
            delay(80);
        }
    }
    steppers.begin();

    delay(STARTUP_SETTLE_MS);
    imu.calibrateGyro();

    digitalWrite(ONBOARD_LED, LOW);
    lastLoopUs = micros();
}

// ------------------------------------------------------------
//  LOOP
// ------------------------------------------------------------
void loop() {
    unsigned long nowUs = micros();
    unsigned long elapsedUs = nowUs - lastLoopUs;
    if (elapsedUs < LOOP_PERIOD_US) return;
    lastLoopUs = nowUs;
    float dt = elapsedUs / 1000000.0f;

    imu.update(dt);
    float pitch = imu.getPitch();

    // ---------- agent connection state machine ----------
    switch (slaveState) {
        case WAITING_AGENT:
            if (millis() - lastPingMs > 500) {
                lastPingMs = millis();
                if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) slaveState = CONNECTING;
            }
            break;
        case CONNECTING:
            slaveState = createEntities() ? RUNNING : WAITING_AGENT;
            if (slaveState == WAITING_AGENT) destroyEntities();
            break;
        case RUNNING:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            if (millis() - lastPingMs > 1000) {
                lastPingMs = millis();
                if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
                    // Agent gone: hard stop, wait for it to return.
                    steppers.setSpeed(0);
                    steppers.disable();
                    motorsEnabled = false;
                    destroyEntities();
                    slaveState = WAITING_AGENT;
                }
            }
            break;
    }

    // ---------- ON-CHIP SAFETY (never moves to ROS) ----------
    bool cmdFresh = (millis() - lastCmdMs) < CMD_TIMEOUT_MS;

    if (motorsEnabled) {
        if (fabsf(pitch) > MAX_TILT_ANGLE ||
            fabsf(imu.getPitchRate()) > MAX_PITCH_RATE_SAFETY) {
            // Fallen or spiking: cut everything locally.
            steppers.setSpeed(0);
            steppers.disable();
            motorsEnabled = false;
        } else if (!cmdFresh) {
            // ROS went quiet — freeze wheels (do NOT disable; the
            // balance node may recover within a few cycles).
            steppers.setSpeeds(0, 0);
        } else {
            steppers.setSpeeds(cmdL, cmdR);
        }
    }

    // ---------- telemetry publish ----------
    if (slaveState == RUNNING && (nowUs - lastPubUs) >= (1000000UL / IMU_PUB_HZ)) {
        lastPubUs = nowUs;
        imuBuf[0] = pitch;
        imuBuf[1] = imu.getPitchRate();
        imuBuf[2] = imu.getYaw();
        imuBuf[3] = (float)steppers.getPositionL();
        imuBuf[4] = (float)steppers.getPositionR();
        imuBuf[5] = dt;
        rcl_publish(&pubImu, &msgImu, NULL);
    }

    // LED: solid = running+enabled, slow blink = agent connected,
    // off = waiting for agent.
    if (motorsEnabled) digitalWrite(ONBOARD_LED, HIGH);
    else if (slaveState == RUNNING) digitalWrite(ONBOARD_LED, (millis() / 500) % 2);
    else digitalWrite(ONBOARD_LED, LOW);
}
