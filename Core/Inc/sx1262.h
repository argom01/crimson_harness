#ifndef SX1262_H
#define SX1262_H

#include <stdint.h>
#include <stdbool.h>

bool sx1262_init(void);
bool sx1262_set_rx(void);
bool sx1262_set_tx(const uint8_t *payload, uint8_t len);
bool sx1262_read_packet(uint8_t *payload, uint8_t max_len, uint8_t *out_len, int16_t *rssi_dbm);
bool sx1262_irq_rx_done(void);
bool sx1262_irq_tx_done(void);
void sx1262_clear_irq(void);
void sx1262_set_rf_switch_rx(void);
void sx1262_set_rf_switch_tx(void);
void sx1262_set_rf_switch_idle(void);

#endif /* SX1262_H */
