#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* MAVLink identity */
#define MAV_SYS_ID                42U
#define MAV_COMP_ID               191U
#define FC_TARGET_SYS             1U
#define FC_TARGET_COMP            1U

/* ArduCopter custom modes */
#define AP_MODE_ALT_HOLD          2U
#define AP_MODE_GUIDED_NOGPS      20U

/* 2.4 GHz LoRa (SX1281) */
#define RF24_FREQUENCY_HZ         2403000000UL
#define RF24_TX_POWER             0x1EU   /* 0 dBm */
#define RF24_SYNC_WORD            0x12U

/* 900 MHz LoRa (SX1262) */
#define RF900_FREQUENCY_HZ        915000000UL
#define RF900_TX_POWER            0x16U   /* 22 dBm with SX1262 PA */
#define RF900_SYNC_WORD           0x34U

/* Hand pose uplink packet */
#define HAND_PACKET_MAGIC         0xC1U
#define HAND_PACKET_SIZE          20U

/* Link health */
#define HAND_LINK_TIMEOUT_MS      500U
#define GS_ACTIVITY_TIMEOUT_MS    2000U
#define RSSI_WEAK_THRESHOLD_DBM   (-95)
#define HAND_CONTROL_HZ           50U
#define HARNESS_HEARTBEAT_HZ      1U

/* Attitude safety limits (degrees) */
#define MAX_ROLL_DEG              45.0f
#define MAX_PITCH_DEG             45.0f
#define MAX_ROLL_RATE_DPS         180.0f
#define MAX_PITCH_RATE_DPS        180.0f
#define QUAT_NORM_TOLERANCE       0.15f

/* Neutral hover thrust (0..1) while hand-controlling attitude */
#define HAND_NEUTRAL_THRUST       0.5f

/* LoRa payload limit */
#define LORA_MAX_PAYLOAD          240U

#endif /* CONFIG_H */
