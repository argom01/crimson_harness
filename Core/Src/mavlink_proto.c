#include "mavlink_proto.h"
#include "config.h"
#include "main.h"

/* CRC extra bytes for used messages (MAVLink common dialect) */
#define CRC_EXTRA_HEARTBEAT             50U
#define CRC_EXTRA_SET_MODE              89U
#define CRC_EXTRA_SET_ATTITUDE_TARGET   49U

static uint8_t tx_seq;

static uint16_t crc_accumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = (uint8_t)(data ^ (uint8_t)(crc & 0xFFU));
    tmp ^= (uint8_t)(tmp << 4);
    return (uint16_t)((crc >> 8) ^ (uint16_t)tmp << 8 ^ (uint16_t)tmp << 3 ^ ((uint16_t)tmp >> 4));
}

static uint16_t mavlink_crc(const uint8_t *payload, uint8_t len, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc = crc_accumulate(payload[i], crc);
    }
    crc = crc_accumulate(crc_extra, crc);
    return crc;
}

static void put_le32(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8) & 0xFFU);
    dst[2] = (uint8_t)((v >> 16) & 0xFFU);
    dst[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void put_le16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void put_float(uint8_t *dst, float v)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = v;
    dst[0] = u.b[0];
    dst[1] = u.b[1];
    dst[2] = u.b[2];
    dst[3] = u.b[3];
}

bool mavlink_frame_build(uint32_t msg_id, const uint8_t *payload, uint8_t payload_len,
                         uint8_t sys_id, uint8_t comp_id, uint8_t seq,
                         mavlink_frame_t *out)
{
    if (out == NULL || payload_len > MAV_MAX_PAYLOAD) {
        return false;
    }

    uint8_t crc_extra = 0U;
    if (msg_id == 0U) {
        crc_extra = CRC_EXTRA_HEARTBEAT;
    } else if (msg_id == 11U) {
        crc_extra = CRC_EXTRA_SET_MODE;
    } else if (msg_id == 82U) {
        crc_extra = CRC_EXTRA_SET_ATTITUDE_TARGET;
    } else {
        return false;
    }

    uint16_t idx = 0U;
    out->buf[idx++] = MAVLINK_STX_V2;
    out->buf[idx++] = payload_len;
    out->buf[idx++] = 0U;
    out->buf[idx++] = 0U;
    out->buf[idx++] = seq;
    out->buf[idx++] = sys_id;
    out->buf[idx++] = comp_id;
    out->buf[idx++] = (uint8_t)(msg_id & 0xFFU);
    out->buf[idx++] = (uint8_t)((msg_id >> 8) & 0xFFU);
    out->buf[idx++] = (uint8_t)((msg_id >> 16) & 0xFFU);

    for (uint8_t i = 0U; i < payload_len; i++) {
        out->buf[idx++] = payload[i];
    }

    uint8_t crc_buf[10U + 255U];
    crc_buf[0] = payload_len;
    crc_buf[1] = 0U;
    crc_buf[2] = 0U;
    crc_buf[3] = seq;
    crc_buf[4] = sys_id;
    crc_buf[5] = comp_id;
    crc_buf[6] = (uint8_t)(msg_id & 0xFFU);
    crc_buf[7] = (uint8_t)((msg_id >> 8) & 0xFFU);
    crc_buf[8] = (uint8_t)((msg_id >> 16) & 0xFFU);
    for (uint8_t i = 0U; i < payload_len; i++) {
        crc_buf[9U + i] = payload[i];
    }

    uint16_t crc = mavlink_crc(crc_buf, (uint8_t)(9U + payload_len), crc_extra);
    put_le16(&out->buf[idx], crc);
    idx += 2U;
    out->len = idx;
    return true;
}

static bool uart_send_frame(const mavlink_frame_t *frame)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)frame->buf, frame->len, 50U) == HAL_OK;
}

bool mavlink_send_heartbeat(uint8_t sys_id, uint8_t comp_id, uint8_t *seq)
{
    uint8_t payload[9];
    put_le32(&payload[0], 0U);
    payload[4] = 6U;   /* MAV_TYPE_ONBOARD_CONTROLLER */
    payload[5] = 8U;   /* MAV_AUTOPILOT_INVALID (companion) */
    payload[6] = 0U;
    payload[7] = 0U;
    payload[8] = 4U;   /* MAV_STATE_ACTIVE */

    mavlink_frame_t frame;
    uint8_t s = (seq != NULL) ? *seq : tx_seq;
    if (!mavlink_frame_build(0U, payload, 9U, sys_id, comp_id, s, &frame)) {
        return false;
    }
    if (seq != NULL) {
        (*seq)++;
    } else {
        tx_seq++;
    }
    return uart_send_frame(&frame);
}

bool mavlink_send_set_mode(uint8_t sys_id, uint8_t comp_id, uint8_t target_sys,
                           uint32_t custom_mode, uint8_t *seq)
{
    uint8_t payload[6];
    put_le32(&payload[0], custom_mode);
    payload[4] = target_sys;
    payload[5] = 1U; /* base_mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED */

    mavlink_frame_t frame;
    uint8_t s = (seq != NULL) ? *seq : tx_seq;
    if (!mavlink_frame_build(11U, payload, 6U, sys_id, comp_id, s, &frame)) {
        return false;
    }
    if (seq != NULL) {
        (*seq)++;
    } else {
        tx_seq++;
    }
    return uart_send_frame(&frame);
}

bool mavlink_send_set_attitude_target(uint8_t sys_id, uint8_t comp_id, uint8_t target_sys,
                                      const float q[4], float thrust, uint8_t *seq)
{
    uint8_t payload[39];
    for (uint8_t i = 0U; i < 39U; i++) {
        payload[i] = 0U;
    }

    put_le32(&payload[0], HAL_GetTick());
    put_float(&payload[4], q[0]);
    put_float(&payload[8], q[1]);
    put_float(&payload[12], q[2]);
    put_float(&payload[16], q[3]);
    /* body rates ignored via type_mask, leave zero */
    put_float(&payload[32], thrust);
    payload[36] = target_sys;
    payload[37] = 1U;
    payload[38] = 0x07U; /* ignore body roll/pitch/yaw rates */

    mavlink_frame_t frame;
    uint8_t s = (seq != NULL) ? *seq : tx_seq;
    if (!mavlink_frame_build(82U, payload, 39U, sys_id, comp_id, s, &frame)) {
        return false;
    }
    if (seq != NULL) {
        (*seq)++;
    } else {
        tx_seq++;
    }
    return uart_send_frame(&frame);
}

void mavlink_stream_init(mavlink_stream_t *stream)
{
    if (stream == NULL) {
        return;
    }
    stream->head = 0U;
    stream->tail = 0U;
    stream->parse_state = 0U;
    stream->frame_len = 0U;
    stream->frame_idx = 0U;
}

static bool build_from_parsed(mavlink_stream_t *stream, mavlink_frame_t *out)
{
    if (stream->frame_idx < 10U) {
        return false;
    }

    uint8_t payload_len = stream->frame_buf[1];
    uint16_t total = (uint16_t)(10U + payload_len + 2U);
    if (stream->frame_idx < total) {
        return false;
    }

    for (uint16_t i = 0U; i < total; i++) {
        out->buf[i] = stream->frame_buf[i];
    }
    out->len = total;
    return true;
}

bool mavlink_stream_push_byte(mavlink_stream_t *stream, uint8_t byte, mavlink_frame_t *out)
{
    if (stream == NULL || out == NULL) {
        return false;
    }

    switch (stream->parse_state) {
    case 0U:
        if (byte == MAVLINK_STX_V2) {
            stream->frame_buf[0] = byte;
            stream->frame_idx = 1U;
            stream->parse_state = 1U;
        }
        break;

    case 1U:
        stream->frame_buf[stream->frame_idx++] = byte;
        if (stream->frame_idx == 2U) {
            stream->frame_len = byte;
            if (stream->frame_len > MAV_MAX_PAYLOAD) {
                stream->parse_state = 0U;
            }
        } else if (stream->frame_idx >= (uint8_t)(10U + stream->frame_len + 2U)) {
            stream->parse_state = 0U;
            return build_from_parsed(stream, out);
        }
        break;

    default:
        stream->parse_state = 0U;
        break;
    }

    return false;
}
