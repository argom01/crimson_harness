#include "sx1262.h"
#include "config.h"
#include "spi_bus.h"
#include "main.h"

#define OPCODE_SET_SLEEP            0x84U
#define OPCODE_SET_STANDBY          0x80U
#define OPCODE_SET_PACKET_TYPE      0x8AU
#define OPCODE_SET_RF_FREQUENCY     0x86U
#define OPCODE_SET_PA_CONFIG        0x95U
#define OPCODE_SET_TX_PARAMS        0x8EU
#define OPCODE_SET_MODULATION_PARAMS 0x8BU
#define OPCODE_SET_PACKET_PARAMS    0x8CU
#define OPCODE_SET_BUFFER_BASE_ADDR 0x8FU
#define OPCODE_SET_DIO_IRQ_PARAMS   0x08U
#define OPCODE_CLR_IRQ_STATUS       0x02U
#define OPCODE_GET_IRQ_STATUS       0x12U
#define OPCODE_GET_RX_BUFFER_STATUS 0x13U
#define OPCODE_GET_PACKET_STATUS    0x14U
#define OPCODE_WRITE_BUFFER         0x0EU
#define OPCODE_READ_BUFFER          0x1EU
#define OPCODE_SET_RX               0x82U
#define OPCODE_SET_TX               0x83U

#define PACKET_TYPE_LORA            0x01U
#define STDBY_RC                    0x00U
#define LORA_SF7                    0x07U
#define LORA_BW_125                 0x04U
#define LORA_CR_4_5                 0x01U
#define IRQ_RX_DONE                 0x0002U
#define IRQ_TX_DONE                 0x0001U

static void wait_not_busy(void)
{
    while (HAL_GPIO_ReadPin(RF900_BUSY_GPIO_Port, RF900_BUSY_Pin) == GPIO_PIN_SET) {
    }
}

static void reset_radio(void)
{
    HAL_GPIO_WritePin(RF900_RST_GPIO_Port, RF900_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RF900_RST_GPIO_Port, RF900_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

static void send_command(uint8_t opcode, const uint8_t *params, uint16_t param_len)
{
    wait_not_busy();
    spi_bus_select(SPI_DEV_RF900);
    spi_bus_write(&opcode, 1U);
    if (param_len > 0U && params != NULL) {
        spi_bus_write(params, param_len);
    }
    spi_bus_deselect(SPI_DEV_RF900);
}

static void read_command(uint8_t opcode, uint8_t *resp, uint16_t resp_len)
{
    wait_not_busy();
    spi_bus_select(SPI_DEV_RF900);
    spi_bus_write(&opcode, 1U);
    uint8_t dummy = 0U;
    spi_bus_read(&dummy, 1U);
    if (resp_len > 0U && resp != NULL) {
        spi_bus_read(resp, resp_len);
    }
    spi_bus_deselect(SPI_DEV_RF900);
}

void sx1262_set_rf_switch_rx(void)
{
    HAL_GPIO_WritePin(RF900_TXEN_GPIO_Port, RF900_TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF900_RXEN_GPIO_Port, RF900_RXEN_Pin, GPIO_PIN_SET);
}

void sx1262_set_rf_switch_tx(void)
{
    HAL_GPIO_WritePin(RF900_RXEN_GPIO_Port, RF900_RXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF900_TXEN_GPIO_Port, RF900_TXEN_Pin, GPIO_PIN_SET);
}

void sx1262_set_rf_switch_idle(void)
{
    HAL_GPIO_WritePin(RF900_TXEN_GPIO_Port, RF900_TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF900_RXEN_GPIO_Port, RF900_RXEN_Pin, GPIO_PIN_RESET);
}

static void set_frequency(uint32_t freq_hz)
{
    uint8_t buf[4];
    uint32_t reg = (uint32_t)((double)freq_hz / (double)32000000ULL * (double)(1ULL << 25));
    buf[0] = (uint8_t)((reg >> 24) & 0xFFU);
    buf[1] = (uint8_t)((reg >> 16) & 0xFFU);
    buf[2] = (uint8_t)((reg >> 8) & 0xFFU);
    buf[3] = (uint8_t)(reg & 0xFFU);
    send_command(OPCODE_SET_RF_FREQUENCY, buf, 4U);
}

static void configure_lora(void)
{
    uint8_t pkt_type = PACKET_TYPE_LORA;
    send_command(OPCODE_SET_PACKET_TYPE, &pkt_type, 1U);

    uint8_t mod[] = {LORA_SF7, LORA_BW_125, LORA_CR_4_5, 0x00U};
    send_command(OPCODE_SET_MODULATION_PARAMS, mod, 4U);

    uint8_t pkt[] = {0x00U, 0x40U, 0x00U, RF900_SYNC_WORD, 0x01U, 0x00U, 0xFFU, 0x00U};
    send_command(OPCODE_SET_PACKET_PARAMS, pkt, 8U);

    uint8_t base[] = {0x00U, 0x00U};
    send_command(OPCODE_SET_BUFFER_BASE_ADDR, base, 2U);

    uint8_t irq[] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    send_command(OPCODE_SET_DIO_IRQ_PARAMS, irq, 8U);

    uint8_t pa[] = {0x04U, 0x07U, 0x00U, 0x01U};
    send_command(OPCODE_SET_PA_CONFIG, pa, 4U);
}

bool sx1262_init(void)
{
    reset_radio();
    sx1262_set_rf_switch_idle();

    uint8_t sleep[] = {0x00U};
    send_command(OPCODE_SET_SLEEP, sleep, 1U);
    HAL_Delay(5);

    uint8_t standby = STDBY_RC;
    send_command(OPCODE_SET_STANDBY, &standby, 1U);
    configure_lora();
    set_frequency(RF900_FREQUENCY_HZ);

    uint8_t tx_params[] = {RF900_TX_POWER, 0x04U};
    send_command(OPCODE_SET_TX_PARAMS, tx_params, 2U);

    sx1262_clear_irq();
    return sx1262_set_rx();
}

bool sx1262_set_rx(void)
{
    sx1262_set_rf_switch_rx();
    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    send_command(OPCODE_SET_RX, timeout, 3U);
    return true;
}

bool sx1262_set_tx(const uint8_t *payload, uint8_t len)
{
    if (payload == NULL || len == 0U) {
        return false;
    }

    sx1262_set_rf_switch_tx();

    wait_not_busy();
    spi_bus_select(SPI_DEV_RF900);
    uint8_t cmd = OPCODE_WRITE_BUFFER;
    uint8_t offset = 0x00U;
    spi_bus_write(&cmd, 1U);
    spi_bus_write(&offset, 1U);
    spi_bus_write(payload, len);
    spi_bus_deselect(SPI_DEV_RF900);

    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    send_command(OPCODE_SET_TX, timeout, 3U);
    return true;
}

bool sx1262_irq_rx_done(void)
{
    uint8_t status[3];
    read_command(OPCODE_GET_IRQ_STATUS, status, 3U);
    uint16_t irq = (uint16_t)((status[1] << 8) | status[2]);
    return (irq & IRQ_RX_DONE) != 0U;
}

bool sx1262_irq_tx_done(void)
{
    uint8_t status[3];
    read_command(OPCODE_GET_IRQ_STATUS, status, 3U);
    uint16_t irq = (uint16_t)((status[1] << 8) | status[2]);
    return (irq & IRQ_TX_DONE) != 0U;
}

void sx1262_clear_irq(void)
{
    uint8_t clr[] = {0xFFU, 0xFFU};
    send_command(OPCODE_CLR_IRQ_STATUS, clr, 2U);
}

bool sx1262_read_packet(uint8_t *payload, uint8_t max_len, uint8_t *out_len, int16_t *rssi_dbm)
{
    if (payload == NULL || out_len == NULL) {
        return false;
    }

    uint8_t rx_status[3];
    read_command(OPCODE_GET_RX_BUFFER_STATUS, rx_status, 3U);
    uint8_t len = rx_status[1];
    if (len == 0U || len > max_len) {
        sx1262_clear_irq();
        sx1262_set_rx();
        return false;
    }

    wait_not_busy();
    spi_bus_select(SPI_DEV_RF900);
    uint8_t cmd = OPCODE_READ_BUFFER;
    uint8_t offset = rx_status[2];
    uint8_t dummy = 0U;
    spi_bus_write(&cmd, 1U);
    spi_bus_read(&dummy, 1U);
    spi_bus_write(&offset, 1U);
    spi_bus_read(payload, len);
    spi_bus_deselect(SPI_DEV_RF900);

    if (rssi_dbm != NULL) {
        uint8_t pkt_status[4];
        read_command(OPCODE_GET_PACKET_STATUS, pkt_status, 4U);
        *rssi_dbm = (int16_t)(-(int16_t)pkt_status[2] / 2);
    }

    *out_len = len;
    sx1262_clear_irq();
    sx1262_set_rx();
    return true;
}
