#include "hand_safety.h"
#include "config.h"
#include "main.h"
#include <math.h>
#include <string.h>

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t seq;
    float q[4]; /* w, x, y, z */
    uint16_t crc16;
} hand_pose_packet_t;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void quat_to_euler_deg(const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    const float roll = atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(1.5707963f, sinp);
    } else {
        pitch = asinf(sinp);
    }

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    const float yaw = atan2f(siny_cosp, cosy_cosp);

    *roll_deg = roll * 57.2957795f;
    *pitch_deg = pitch * 57.2957795f;
    *yaw_deg = yaw * 57.2957795f;
}

static hand_safety_reason_t check_quaternion(hand_pose_state_t *state, const float q[4], uint32_t now_ms)
{
    const float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    const float norm = sqrtf(norm_sq);
    if (fabsf(norm - 1.0f) > QUAT_NORM_TOLERANCE) {
        return HAND_SAFE_BAD_QUAT;
    }

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    quat_to_euler_deg(q, &roll_deg, &pitch_deg, &yaw_deg);

    if (fabsf(roll_deg) > MAX_ROLL_DEG || fabsf(pitch_deg) > MAX_PITCH_DEG) {
        return HAND_SAFE_TILT_EXCESS;
    }

    if (state->valid) {
        float prev_roll = 0.0f;
        float prev_pitch = 0.0f;
        float prev_yaw = 0.0f;
        quat_to_euler_deg(state->q, &prev_roll, &prev_pitch, &prev_yaw);

        const float dt_s = (float)(now_ms - state->last_rx_ms) / 1000.0f;
        if (dt_s > 0.001f) {
            const float roll_rate = fabsf(roll_deg - prev_roll) / dt_s;
            const float pitch_rate = fabsf(pitch_deg - prev_pitch) / dt_s;
            if (roll_rate > MAX_ROLL_RATE_DPS || pitch_rate > MAX_PITCH_RATE_DPS) {
                return HAND_SAFE_RATE_EXCESS;
            }
        }
    }

    (void)yaw_deg;
    return HAND_SAFE_OK;
}

void hand_pose_init(hand_pose_state_t *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->last_reason = HAND_SAFE_LINK_TIMEOUT;
    state->rssi_dbm = -127;
}

bool hand_pose_feed(hand_pose_state_t *state, const uint8_t *packet, uint8_t len,
                    int16_t rssi_dbm, uint32_t now_ms)
{
    if (state == NULL || packet == NULL || len < HAND_PACKET_SIZE) {
        return false;
    }

    const hand_pose_packet_t *pkt = (const hand_pose_packet_t *)packet;
    if (pkt->magic != HAND_PACKET_MAGIC) {
        state->last_reason = HAND_SAFE_BAD_MAGIC;
        return false;
    }

    const uint16_t expected_crc = crc16_ccitt(packet, (uint16_t)(HAND_PACKET_SIZE - 2U));
    if (expected_crc != pkt->crc16) {
        state->last_reason = HAND_SAFE_BAD_CRC;
        return false;
    }

    if (rssi_dbm < RSSI_WEAK_THRESHOLD_DBM) {
        state->last_reason = HAND_SAFE_RSSI_WEAK;
        state->rssi_dbm = rssi_dbm;
        return false;
    }

    const hand_safety_reason_t quat_reason = check_quaternion(state, pkt->q, now_ms);
    if (quat_reason != HAND_SAFE_OK) {
        state->last_reason = quat_reason;
        return false;
    }

    state->q[0] = pkt->q[0];
    state->q[1] = pkt->q[1];
    state->q[2] = pkt->q[2];
    state->q[3] = pkt->q[3];
    state->seq = pkt->seq;
    state->rssi_dbm = rssi_dbm;
    state->last_rx_ms = now_ms;
    state->valid = true;
    state->last_reason = HAND_SAFE_OK;
    return true;
}

bool hand_pose_link_ok(const hand_pose_state_t *state, uint32_t now_ms)
{
    if (state == NULL || !state->valid) {
        return false;
    }
    return (now_ms - state->last_rx_ms) <= HAND_LINK_TIMEOUT_MS;
}

hand_safety_reason_t hand_pose_last_reason(const hand_pose_state_t *state)
{
    if (state == NULL) {
        return HAND_SAFE_LINK_TIMEOUT;
    }
    if (!hand_pose_link_ok(state, HAL_GetTick())) {
        return HAND_SAFE_LINK_TIMEOUT;
    }
    return state->last_reason;
}

uint16_t hand_pose_packet_crc(const uint8_t *packet, uint8_t len)
{
    if (packet == NULL || len < HAND_PACKET_SIZE) {
        return 0U;
    }
    return crc16_ccitt(packet, (uint16_t)(HAND_PACKET_SIZE - 2U));
}
