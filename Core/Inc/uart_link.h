#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdint.h>
#include <stdbool.h>

/* DMA-backed, non-blocking FC UART link (USART1).
   Call uart_link_init() after the UART peripheral is initialized and
   uart_link_poll() once per main-loop pass. */

void uart_link_init(void);
void uart_link_poll(void);

/* Queue a complete message for transmission. Returns false (and drops the
   whole message, keeping the MAVLink stream aligned) if it does not fit. */
bool uart_link_write(const uint8_t *data, uint16_t len);

/* Fetch one received byte; returns false when the RX buffer is empty. */
bool uart_link_read_byte(uint8_t *byte);

#endif /* UART_LINK_H */
