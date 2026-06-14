#ifndef MAVLINK_PROTO_H
#define MAVLINK_PROTO_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MAVLINK_STX_V2            0xFDU

typedef struct {
    uint8_t buf[MAVLINK_STX_V2 + 12U + 255U + 2U];
    uint16_t len;
} mavlink_frame_t;

typedef struct {
    uint8_t data[512];
    uint16_t head;
    uint16_t tail;
    uint8_t parse_state;
    uint8_t frame_len;
    uint8_t frame_idx;
    uint8_t frame_buf[280];
} mavlink_stream_t;

void mavlink_stream_init(mavlink_stream_t *stream);
bool mavlink_stream_push_byte(mavlink_stream_t *stream, uint8_t byte, mavlink_frame_t *out);
bool mavlink_frame_build(uint32_t msg_id, const uint8_t *payload, uint8_t payload_len,
                         uint8_t sys_id, uint8_t comp_id, uint8_t seq,
                         mavlink_frame_t *out);

bool mavlink_send_set_mode(uint8_t sys_id, uint8_t comp_id, uint8_t target_sys,
                           uint32_t custom_mode, uint8_t *seq);
bool mavlink_send_set_attitude_target(uint8_t sys_id, uint8_t comp_id, uint8_t target_sys,
                                      const float q[4], float thrust, uint8_t *seq);
bool mavlink_send_heartbeat(uint8_t sys_id, uint8_t comp_id, uint8_t *seq);

#endif /* MAVLINK_PROTO_H */
