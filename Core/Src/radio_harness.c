#include "radio_harness.h"
#include "config.h"
#include "hand_safety.h"
#include "spi_bus.h"
#include "sx1262.h"
#include "sx1281.h"
#include "main.h"
#include "mavlink.h"
#include <string.h>

typedef enum {
    HARNESS_STATE_FAILSAFE = 0,
    HARNESS_STATE_HAND_CONTROL,
    HARNESS_STATE_GS_PASSTHROUGH,
} harness_state_t;

static harness_state_t harness_state = HARNESS_STATE_FAILSAFE;
static hand_pose_state_t hand_state;
static mavlink_status_t uart_mavlink_status;
static mavlink_status_t gs_mavlink_status;
static uint32_t last_gs_activity_ms;
static uint32_t last_hand_tx_ms;
static uint32_t last_heartbeat_ms;
static uint32_t last_mode_tx_ms;
static bool failsafe_mode_sent;
static bool guided_mode_sent;

static void enter_state(harness_state_t new_state)
{
    harness_state = new_state;
    if (new_state == HARNESS_STATE_HAND_CONTROL) {
        guided_mode_sent = false;
    } else if (new_state == HARNESS_STATE_FAILSAFE) {
        failsafe_mode_sent = false;
        guided_mode_sent = false;
    }
}

static bool mavlink_uart_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    return HAL_UART_Transmit(&huart1, buf, len, 50U) == HAL_OK;
}

static bool mavlink_send_set_mode(uint32_t custom_mode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                              FC_TARGET_SYS, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode);
    return mavlink_uart_send(&msg);
}

static bool mavlink_send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                               MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                               0U, 0U, MAV_STATE_ACTIVE);
    return mavlink_uart_send(&msg);
}

static bool mavlink_send_set_attitude_target(const float q[4], float thrust)
{
    const float thrust_body[3] = {0.0f, 0.0f, 0.0f};
    mavlink_message_t msg;

    mavlink_msg_set_attitude_target_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                         HAL_GetTick(), FC_TARGET_SYS, FC_TARGET_COMP,
                                         0x07U, q, 0.0f, 0.0f, 0.0f, thrust, thrust_body);
    return mavlink_uart_send(&msg);
}

static void forward_mavlink_to_uart(const mavlink_message_t *msg)
{
    (void)mavlink_uart_send(msg);
}

static void forward_mavlink_to_gs(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    if (len > LORA_MAX_PAYLOAD) {
        return;
    }

    sx1262_set_tx(buf, (uint8_t)len);

    const uint32_t deadline = HAL_GetTick() + 500U;
    while (!sx1262_irq_tx_done()) {
        if (HAL_GetTick() >= deadline) {
            break;
        }
    }

    sx1262_clear_irq();
    sx1262_set_rx();
}

static void poll_rf24(void)
{
    if (!sx1281_irq_rx_done()) {
        return;
    }

    uint8_t buf[LORA_MAX_PAYLOAD];
    uint8_t len = 0U;
    int16_t rssi = -127;
    int8_t snr = 0;

    if (!sx1281_read_packet(buf, sizeof(buf), &len, &rssi, &snr)) {
        return;
    }

    (void)snr;
    (void)hand_pose_feed(&hand_state, buf, len, rssi, HAL_GetTick());
}

static void poll_rf900(void)
{
    if (!sx1262_irq_rx_done()) {
        return;
    }

    uint8_t buf[LORA_MAX_PAYLOAD];
    uint8_t len = 0U;
    int16_t rssi = -127;

    if (!sx1262_read_packet(buf, sizeof(buf), &len, &rssi)) {
        return;
    }

    (void)rssi;
    last_gs_activity_ms = HAL_GetTick();

    mavlink_message_t msg;
    for (uint8_t i = 0U; i < len; i++) {
        if (!mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &gs_mavlink_status)) {
            continue;
        }

        if (harness_state == HARNESS_STATE_GS_PASSTHROUGH ||
            harness_state == HARNESS_STATE_FAILSAFE) {
            forward_mavlink_to_uart(&msg);
        }
    }
}

static void poll_uart(void)
{
    uint8_t byte;
    mavlink_message_t msg;

    while (HAL_UART_Receive(&huart1, &byte, 1U, 0U) == HAL_OK) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &uart_mavlink_status)) {
            continue;
        }

        if (harness_state == HARNESS_STATE_GS_PASSTHROUGH ||
            harness_state == HARNESS_STATE_HAND_CONTROL ||
            harness_state == HARNESS_STATE_FAILSAFE) {
            forward_mavlink_to_gs(&msg);
        }
    }
}

static bool gs_link_active(uint32_t now_ms)
{
    return (now_ms - last_gs_activity_ms) <= GS_ACTIVITY_TIMEOUT_MS;
}

static void update_arbitration(uint32_t now_ms)
{
    const bool hand_ok = hand_pose_link_ok(&hand_state, now_ms) &&
                         hand_state.last_reason == HAND_SAFE_OK;
    const bool gs_active = gs_link_active(now_ms);

    if (gs_active) {
        if (harness_state != HARNESS_STATE_GS_PASSTHROUGH) {
            enter_state(HARNESS_STATE_GS_PASSTHROUGH);
        }
        return;
    }

    if (hand_ok) {
        if (harness_state != HARNESS_STATE_HAND_CONTROL) {
            enter_state(HARNESS_STATE_HAND_CONTROL);
        }
        return;
    }

    if (harness_state != HARNESS_STATE_FAILSAFE) {
        enter_state(HARNESS_STATE_FAILSAFE);
    }
}

static void service_failsafe(uint32_t now_ms)
{
    if (!failsafe_mode_sent || (now_ms - last_mode_tx_ms) > 3000U) {
        if (mavlink_send_set_mode(AP_MODE_ALT_HOLD)) {
            failsafe_mode_sent = true;
            last_mode_tx_ms = now_ms;
        }
    }
}

static void service_hand_control(uint32_t now_ms)
{
    const uint32_t hand_period_ms = 1000U / HAND_CONTROL_HZ;

    if (!guided_mode_sent || (now_ms - last_mode_tx_ms) > 3000U) {
        if (mavlink_send_set_mode(AP_MODE_GUIDED_NOGPS)) {
            guided_mode_sent = true;
            last_mode_tx_ms = now_ms;
        }
    }

    if ((now_ms - last_hand_tx_ms) >= hand_period_ms && hand_state.valid) {
        if (mavlink_send_set_attitude_target(hand_state.q, HAND_NEUTRAL_THRUST)) {
            last_hand_tx_ms = now_ms;
        }
    }
}

static void service_heartbeat(uint32_t now_ms)
{
    const uint32_t hb_period_ms = 1000U / HARNESS_HEARTBEAT_HZ;
    if ((now_ms - last_heartbeat_ms) >= hb_period_ms) {
        if (mavlink_send_heartbeat()) {
            last_heartbeat_ms = now_ms;
        }
    }
}

void radio_harness_init(void)
{
    spi_bus_init();
    hand_pose_init(&hand_state);
    memset(&uart_mavlink_status, 0, sizeof(uart_mavlink_status));
    memset(&gs_mavlink_status, 0, sizeof(gs_mavlink_status));

    last_gs_activity_ms = 0U;
    last_hand_tx_ms = 0U;
    last_heartbeat_ms = 0U;
    last_mode_tx_ms = 0U;
    failsafe_mode_sent = false;
    guided_mode_sent = false;

    if (!sx1281_init()) {
        Error_Handler();
    }
    if (!sx1262_init()) {
        Error_Handler();
    }

    enter_state(HARNESS_STATE_FAILSAFE);
}

void radio_harness_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();

    poll_rf24();
    poll_rf900();
    poll_uart();
    update_arbitration(now_ms);
    service_heartbeat(now_ms);

    switch (harness_state) {
    case HARNESS_STATE_HAND_CONTROL:
        service_hand_control(now_ms);
        break;
    case HARNESS_STATE_GS_PASSTHROUGH:
        break;
    case HARNESS_STATE_FAILSAFE:
    default:
        service_failsafe(now_ms);
        break;
    }
}
