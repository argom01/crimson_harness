#ifndef SPI_BUS_H
#define SPI_BUS_H

#include "main.h"
#include <stdint.h>

typedef enum {
    SPI_DEV_RF24 = 0,
    SPI_DEV_RF900,
    SPI_DEV_COUNT
} spi_device_t;

void spi_bus_init(void);
void spi_bus_select(spi_device_t dev);
void spi_bus_deselect(spi_device_t dev);
HAL_StatusTypeDef spi_bus_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
HAL_StatusTypeDef spi_bus_write(const uint8_t *tx, uint16_t len);
HAL_StatusTypeDef spi_bus_read(uint8_t *rx, uint16_t len);

#endif /* SPI_BUS_H */
