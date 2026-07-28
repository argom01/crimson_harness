#ifndef HAND_SAFETY_H
#define HAND_SAFETY_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    HAND_SAFE_OK = 0,
    HAND_SAFE_BAD_MAGIC,
    HAND_SAFE_BAD_CRC,
    HAND_SAFE_BAD_QUAT,
    HAND_SAFE_TILT_EXCESS,
    HAND_SAFE_RATE_EXCESS,
    HAND_SAFE_LINK_TIMEOUT,
    HAND_SAFE_RSSI_WEAK,
} hand_safety_reason_t;

typedef struct {
    float q[4];
    uint8_t seq;
    int16_t rssi_dbm;
    uint32_t last_rx_ms;
    bool valid;
    hand_safety_reason_t last_reason;
    /* Hysteresis: the link engages after HAND_ENGAGE_GOOD_COUNT consecutive
       valid packets and disengages after HAND_DISENGAGE_BAD_COUNT
       consecutive rejects or a gap longer than HAND_LINK_TIMEOUT_MS. */
    uint8_t good_streak;
    uint8_t reject_streak;
    bool engaged;
} hand_pose_state_t;

void hand_pose_init(hand_pose_state_t *state);
bool hand_pose_feed(hand_pose_state_t *state, const uint8_t *packet, uint8_t len,
                    int16_t rssi_dbm, uint32_t now_ms);
bool hand_pose_link_ok(const hand_pose_state_t *state, uint32_t now_ms);

#endif /* HAND_SAFETY_H */
