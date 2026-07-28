#ifndef APP_ERROR_H
#define APP_ERROR_H

#include <stdint.h>

typedef enum {
    APP_ERR_NONE = 0,
    APP_ERR_HAL,        /* HAL / generated-code Error_Handler */
    APP_ERR_FAULT,      /* CPU fault (HardFault, BusFault, ...) */
    APP_ERR_SPI,        /* SPI transaction failed */
    APP_ERR_RF24_BUSY,  /* SX1281 BUSY stuck high */
    APP_ERR_RF24_INIT,  /* SX1281 (re)init failed */
    APP_ERR_RF900_BUSY, /* SX1262 BUSY stuck high */
    APP_ERR_RF900_INIT, /* SX1262 (re)init failed */
    APP_ERR_UART_TX,    /* UART transmit failed or message dropped */
    APP_ERR_UART_RX,    /* UART reception aborted / re-armed */
    APP_ERR_COUNT
} app_error_t;

/* Record a recoverable error and keep running. */
void app_error_report(app_error_t err);

/* Record the reason in a backup register and reset the MCU. Never returns. */
void app_error_fatal(app_error_t err);

uint16_t app_error_count(app_error_t err);
app_error_t app_error_last(void);
uint32_t app_error_last_ms(void);

/* Reason for the previous self-reset, APP_ERR_NONE if the last boot was
   clean. Clears the stored value, so only the first call reports it. */
app_error_t app_error_boot_fault(void);

#endif /* APP_ERROR_H */
