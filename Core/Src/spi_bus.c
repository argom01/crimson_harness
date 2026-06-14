#include "spi_bus.h"
#include "main.h"

static GPIO_TypeDef *const nss_ports[SPI_DEV_COUNT] = {
    [SPI_DEV_RF24]  = NSS_RF24_GPIO_Port,
    [SPI_DEV_RF900] = NSS_RF900_GPIO_Port,
};

static const uint16_t nss_pins[SPI_DEV_COUNT] = {
    [SPI_DEV_RF24]  = NSS_RF24_Pin,
    [SPI_DEV_RF900] = NSS_RF900_Pin,
};

static spi_device_t active_dev = SPI_DEV_COUNT;

void spi_bus_init(void)
{
    spi_bus_deselect(SPI_DEV_RF24);
    spi_bus_deselect(SPI_DEV_RF900);
    active_dev = SPI_DEV_COUNT;
}

void spi_bus_select(spi_device_t dev)
{
    if (dev >= SPI_DEV_COUNT) {
        return;
    }

    if (active_dev < SPI_DEV_COUNT && active_dev != dev) {
        HAL_GPIO_WritePin(nss_ports[active_dev], nss_pins[active_dev], GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(nss_ports[dev], nss_pins[dev], GPIO_PIN_RESET);
    active_dev = dev;
}

void spi_bus_deselect(spi_device_t dev)
{
    if (dev >= SPI_DEV_COUNT) {
        return;
    }

    HAL_GPIO_WritePin(nss_ports[dev], nss_pins[dev], GPIO_PIN_SET);
    if (active_dev == dev) {
        active_dev = SPI_DEV_COUNT;
    }
}

HAL_StatusTypeDef spi_bus_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100U);
}

HAL_StatusTypeDef spi_bus_write(const uint8_t *tx, uint16_t len)
{
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, 100U);
}

HAL_StatusTypeDef spi_bus_read(uint8_t *rx, uint16_t len)
{
    return HAL_SPI_Receive(&hspi1, rx, len, 100U);
}
