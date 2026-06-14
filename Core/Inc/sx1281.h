#ifndef SX1281_H
#define SX1281_H

#include <stdint.h>
#include <stdbool.h>

bool sx1281_init(void);
bool sx1281_set_rx(void);
bool sx1281_set_tx(const uint8_t *payload, uint8_t len);
bool sx1281_read_packet(uint8_t *payload, uint8_t max_len, uint8_t *out_len, int16_t *rssi_dbm, int8_t *snr_db);
bool sx1281_irq_rx_done(void);
void sx1281_clear_irq(void);

#endif /* SX1281_H */
