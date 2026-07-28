#include "radio_harness.h"
#include "app_error.h"
#include "config.h"
#include "hand_safety.h"
#include "spi_bus.h"
#include "sx1262.h"
#include "sx1281.h"
#include "uart_link.h"
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
static bool gs_seen;
static uint32_t last_hand_tx_ms;
static uint32_t last_heartbeat_ms;
static uint32_t last_mode_tx_ms;
static uint32_t last_rf24_reinit_ms;
static uint32_t last_rf900_reinit_ms;

/* Flight mode commanding is closed-loop: SET_MODE is re-sent until the
   FC's heartbeat reports the target custom_mode. Once confirmed we stop
   commanding, so a mode change made by the FC itself (its own failsafe)
   or by a pilot is not fought. */
static uint32_t mode_target;
static bool mode_confirmed;
static uint32_t fc_custom_mode;
static bool fc_mode_known;
static uint32_t last_fc_heartbeat_ms;

/* 900 MHz downlink TX queue: telemetry is sent without ever blocking the
   control loop. A full payload at SF7/BW125 is ~370 ms of airtime, so the
   queue overflows under load; oldest messages are dropped (telemetry is
   lossy by design, the FC re-sends periodically). */
#define GS_TX_QUEUE_LEN   3U
#define GS_TX_TIMEOUT_MS  500U

static uint8_t gs_tx_buf[GS_TX_QUEUE_LEN][LORA_MAX_PAYLOAD];
static uint8_t gs_tx_len[GS_TX_QUEUE_LEN];
static uint8_t gs_tx_head; /* next slot to transmit */
static uint8_t gs_tx_count;
static bool gs_tx_active;
static uint32_t gs_tx_start_ms;

static void begin_mode_command(uint32_t target_custom_mode)
{
    mode_target = target_custom_mode;
    mode_confirmed = false;
    /* Backdate so the first SET_MODE goes out on the next service pass */
    last_mode_tx_ms = HAL_GetTick() - MODE_CMD_RETRY_MS;
}

static void enter_state(harness_state_t new_state)
{
    harness_state = new_state;
    if (new_state == HARNESS_STATE_HAND_CONTROL) {
        begin_mode_command(AP_MODE_GUIDED_NOGPS);
    } else if (new_state == HARNESS_STATE_FAILSAFE) {
        begin_mode_command(AP_MODE_ALT_HOLD);
    } else {
        /* GS passthrough: the ground station owns the mode */
        mode_confirmed = true;
    }
}

static bool mavlink_uart_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    return uart_link_write(buf, len);
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
    if (len == 0U || len > LORA_MAX_PAYLOAD) {
        return;
    }

    if (gs_tx_count == GS_TX_QUEUE_LEN) {
        /* Queue full: drop the oldest message to keep telemetry fresh */
        gs_tx_head = (uint8_t)((gs_tx_head + 1U) % GS_TX_QUEUE_LEN);
        gs_tx_count--;
    }

    const uint8_t slot = (uint8_t)((gs_tx_head + gs_tx_count) % GS_TX_QUEUE_LEN);
    memcpy(gs_tx_buf[slot], buf, len);
    gs_tx_len[slot] = (uint8_t)len;
    gs_tx_count++;
}

static void service_gs_tx(uint32_t now_ms)
{
    if (!sx1262_healthy()) {
        /* Re-init path puts the radio back into RX; abandon any TX */
        gs_tx_active = false;
        return;
    }

    if (gs_tx_active) {
        if (sx1262_irq_tx_done() || (now_ms - gs_tx_start_ms) > GS_TX_TIMEOUT_MS) {
            (void)sx1262_clear_irq();
            (void)sx1262_set_rx();
            gs_tx_active = false;
        }
        return;
    }

    if (gs_tx_count == 0U) {
        return;
    }

    const uint8_t slot = gs_tx_head;
    gs_tx_head = (uint8_t)((gs_tx_head + 1U) % GS_TX_QUEUE_LEN);
    gs_tx_count--;

    if (sx1262_set_tx(gs_tx_buf[slot], gs_tx_len[slot])) {
        gs_tx_active = true;
        gs_tx_start_ms = now_ms;
    }
}

/* A radio that keeps failing SPI transactions or holds BUSY high is dead;
   periodically try a full reset + re-init instead of hammering it. */
static void service_radio_health(uint32_t now_ms)
{
    if (!sx1281_healthy() && (now_ms - last_rf24_reinit_ms) >= RADIO_REINIT_INTERVAL_MS) {
        last_rf24_reinit_ms = now_ms;
        if (!sx1281_init()) {
            app_error_report(APP_ERR_RF24_INIT);
        }
    }
    if (!sx1262_healthy() && (now_ms - last_rf900_reinit_ms) >= RADIO_REINIT_INTERVAL_MS) {
        last_rf900_reinit_ms = now_ms;
        if (!sx1262_init()) {
            app_error_report(APP_ERR_RF900_INIT);
        }
    }
}

static void poll_rf24(void)
{
    if (!sx1281_healthy() || !sx1281_irq_rx_done()) {
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
    if (gs_tx_active || !sx1262_healthy() || !sx1262_irq_rx_done()) {
        return;
    }

    uint8_t buf[LORA_MAX_PAYLOAD];
    uint8_t len = 0U;
    int16_t rssi = -127;

    if (!sx1262_read_packet(buf, sizeof(buf), &len, &rssi)) {
        return;
    }

    (void)rssi;

    mavlink_message_t msg;
    for (uint8_t i = 0U; i < len; i++) {
        if (!mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &gs_mavlink_status)) {
            continue;
        }

        /* Only a fully parsed MAVLink message counts as GS activity; raw
           RF traffic (noise, foreign LoRa) must not hold us in passthrough */
        last_gs_activity_ms = HAL_GetTick();
        gs_seen = true;

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

    while (uart_link_read_byte(&byte)) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &uart_mavlink_status)) {
            continue;
        }

        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
            msg.sysid == FC_TARGET_SYS && msg.compid == FC_TARGET_COMP) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            fc_custom_mode = hb.custom_mode;
            fc_mode_known = true;
            last_fc_heartbeat_ms = HAL_GetTick();
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
    /* gs_seen guards the boot window where last_gs_activity_ms == 0 would
       otherwise make the GS look active for the first timeout period */
    return gs_seen && (now_ms - last_gs_activity_ms) <= GS_ACTIVITY_TIMEOUT_MS;
}

static void update_arbitration(uint32_t now_ms)
{
    /* Engagement hysteresis lives in hand_safety; a single rejected packet
       no longer flaps the harness out of hand control. */
    const bool hand_ok = hand_pose_link_ok(&hand_state, now_ms);
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

static void service_mode_command(uint32_t now_ms)
{
    if (mode_confirmed) {
        return;
    }
    if (fc_mode_known && fc_custom_mode == mode_target) {
        mode_confirmed = true;
        return;
    }
    if ((now_ms - last_mode_tx_ms) >= MODE_CMD_RETRY_MS) {
        if (mavlink_send_set_mode(mode_target)) {
            last_mode_tx_ms = now_ms;
        }
    }
}

static void service_failsafe(uint32_t now_ms)
{
    service_mode_command(now_ms);
}

static void service_hand_control(uint32_t now_ms)
{
    const uint32_t hand_period_ms = 1000U / HAND_CONTROL_HZ;

    service_mode_command(now_ms);

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
    uart_link_init();
    hand_pose_init(&hand_state);
    memset(&uart_mavlink_status, 0, sizeof(uart_mavlink_status));
    memset(&gs_mavlink_status, 0, sizeof(gs_mavlink_status));

    last_gs_activity_ms = 0U;
    gs_seen = false;
    last_hand_tx_ms = 0U;
    last_heartbeat_ms = 0U;
    last_mode_tx_ms = 0U;
    last_rf24_reinit_ms = 0U;
    last_rf900_reinit_ms = 0U;
    fc_custom_mode = 0U;
    fc_mode_known = false;
    last_fc_heartbeat_ms = 0U;
    gs_tx_head = 0U;
    gs_tx_count = 0U;
    gs_tx_active = false;
    gs_tx_start_ms = 0U;

    /* A dead radio must not brick the harness: the other link stays up and
       service_radio_health() keeps retrying the failed one. */
    if (!sx1281_init()) {
        app_error_report(APP_ERR_RF24_INIT);
    }
    if (!sx1262_init()) {
        app_error_report(APP_ERR_RF900_INIT);
    }

    enter_state(HARNESS_STATE_FAILSAFE);
}

void radio_harness_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();

    service_radio_health(now_ms);
    poll_rf24();
    poll_rf900();
    poll_uart();
    uart_link_poll();
    service_gs_tx(now_ms);
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
