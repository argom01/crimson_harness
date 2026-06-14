#include "sx1281.h"
#include "config.h"
#include "spi_bus.h"
#include "main.h"

/* Semtech SX1281 command opcodes */
#define RADIO_RESET               0xC0U
#define RADIO_SET_STANDBY         0x80U
#define RADIO_SET_PACKETTYPE      0x8AU
#define RADIO_SET_RFFREQUENCY     0x86U
#define RADIO_SET_TXPARAMS        0x8EU
#define RADIO_SET_MODULATIONPARAMS 0x8BU
#define RADIO_SET_PACKETPARAMS    0x8CU
#define RADIO_SET_BUFFERBASEADDRESS 0x8FU
#define RADIO_SET_DIOIRQPARAMS    0x8DU
#define RADIO_CLR_IRQSTATUS       0x97U
#define RADIO_GET_IRQSTATUS       0x15U
#define RADIO_GET_RXBUFFERSTATUS  0x17U
#define RADIO_GET_PACKETSTATUS    0x1DU
#define RADIO_GET_STATUS          0xC0U
#define RADIO_WRITE_BUFFER        0x0EU
#define RADIO_READ_BUFFER         0x1EU
#define RADIO_SET_RX              0x82U
#define RADIO_SET_TX              0x83U

#define PACKET_TYPE_LORA          0x01U
#define STDBY_XOSC                0x01U
#define LORA_BW_1600              0x0BU
#define LORA_SF5                  0x05U
#define LORA_CR_4_5               0x01U
#define IRQ_RX_DONE               0x0040U
#define IRQ_TX_DONE               0x0001U
#define IRQ_ALL                   0xFFFFU

static void wait_not_busy(void)
{
    while (HAL_GPIO_ReadPin(RF24_BUSY_GPIO_Port, RF24_BUSY_Pin) == GPIO_PIN_SET) {
    }
}

static void reset_radio(void)
{
    HAL_GPIO_WritePin(RF24_RST_GPIO_Port, RF24_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RF24_RST_GPIO_Port, RF24_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

static void send_command(uint8_t cmd, const uint8_t *params, uint16_t param_len)
{
    wait_not_busy();
    spi_bus_select(SPI_DEV_RF24);
    spi_bus_write(&cmd, 1U);
    if (param_len > 0U && params != NULL) {
        spi_bus_write(params, param_len);
    }
    spi_bus_deselect(SPI_DEV_RF24);
}

static void get_status(uint8_t cmd, uint8_t *resp, uint16_t resp_len)
{
    wait_not_busy();
    spi_bus_select(SPI_DEV_RF24);
    spi_bus_write(&cmd, 1U);
    if (resp_len > 0U && resp != NULL) {
        spi_bus_read(resp, resp_len);
    }
    spi_bus_deselect(SPI_DEV_RF24);
}

static void set_frequency(uint32_t freq_hz)
{
    uint8_t buf[3];
    uint32_t reg = (uint32_t)((double)freq_hz / (double)52000000ULL * (double)(1ULL << 25));
    buf[0] = (uint8_t)((reg >> 16) & 0xFFU);
    buf[1] = (uint8_t)((reg >> 8) & 0xFFU);
    buf[2] = (uint8_t)(reg & 0xFFU);
    send_command(RADIO_SET_RFFREQUENCY, buf, 3U);
}

static void configure_lora(void)
{
    uint8_t pkt_type = PACKET_TYPE_LORA;
    send_command(RADIO_SET_PACKETTYPE, &pkt_type, 1U);

    uint8_t mod[] = {LORA_SF5, LORA_BW_1600, LORA_CR_4_5};
    send_command(RADIO_SET_MODULATIONPARAMS, mod, 3U);

    uint8_t pkt[] = {0x00U, 0x40U, 0x00U, RF24_SYNC_WORD, 0x01U, 0x00U, 0xFFU};
    send_command(RADIO_SET_PACKETPARAMS, pkt, 7U);

    uint8_t base[] = {0x00U, 0x00U};
    send_command(RADIO_SET_BUFFERBASEADDRESS, base, 2U);

    uint8_t irq[] = {0x00U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    send_command(RADIO_SET_DIOIRQPARAMS, irq, 8U);
}

bool sx1281_init(void)
{
    reset_radio();
    wait_not_busy();

    uint8_t standby = STDBY_XOSC;
    send_command(RADIO_SET_STANDBY, &standby, 1U);
    configure_lora();
    set_frequency(RF24_FREQUENCY_HZ);

    uint8_t tx_params[] = {RF24_TX_POWER, 0x02U};
    send_command(RADIO_SET_TXPARAMS, tx_params, 2U);

    sx1281_clear_irq();
    return sx1281_set_rx();
}

bool sx1281_set_rx(void)
{
    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    send_command(RADIO_SET_RX, timeout, 3U);
    return true;
}

bool sx1281_set_tx(const uint8_t *payload, uint8_t len)
{
    if (payload == NULL || len == 0U) {
        return false;
    }

    wait_not_busy();
    spi_bus_select(SPI_DEV_RF24);
    uint8_t cmd = RADIO_WRITE_BUFFER;
    uint8_t offset = 0x00U;
    spi_bus_write(&cmd, 1U);
    spi_bus_write(&offset, 1U);
    spi_bus_write(payload, len);
    spi_bus_deselect(SPI_DEV_RF24);

    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    send_command(RADIO_SET_TX, timeout, 3U);
    return true;
}

bool sx1281_irq_rx_done(void)
{
    uint8_t status[3];
    get_status(RADIO_GET_IRQSTATUS, status, 3U);
    uint16_t irq = (uint16_t)((status[1] << 8) | status[2]);
    return (irq & IRQ_RX_DONE) != 0U;
}

void sx1281_clear_irq(void)
{
    uint8_t clr[] = {0xFFU, 0xFFU};
    send_command(RADIO_CLR_IRQSTATUS, clr, 2U);
}

bool sx1281_read_packet(uint8_t *payload, uint8_t max_len, uint8_t *out_len, int16_t *rssi_dbm, int8_t *snr_db)
{
    if (payload == NULL || out_len == NULL) {
        return false;
    }

    uint8_t rx_status[3];
    get_status(RADIO_GET_RXBUFFERSTATUS, rx_status, 3U);
    uint8_t len = rx_status[1];
    if (len == 0U || len > max_len) {
        sx1281_clear_irq();
        sx1281_set_rx();
        return false;
    }

    wait_not_busy();
    spi_bus_select(SPI_DEV_RF24);
    uint8_t tx[2U + 255U];
    uint8_t rx[2U + 255U];
    tx[0] = RADIO_READ_BUFFER;
    tx[1] = rx_status[2];
    for (uint8_t i = 0U; i < len; i++) {
        tx[2U + i] = 0U;
    }
    spi_bus_transfer(tx, rx, (uint16_t)(2U + len));
    for (uint8_t i = 0U; i < len; i++) {
        payload[i] = rx[2U + i];
    }
    spi_bus_deselect(SPI_DEV_RF24);

    if (rssi_dbm != NULL || snr_db != NULL) {
        uint8_t pkt_status[4];
        get_status(RADIO_GET_PACKETSTATUS, pkt_status, 4U);
        if (rssi_dbm != NULL) {
            *rssi_dbm = (int16_t)(-(int16_t)pkt_status[2] / 2);
        }
        if (snr_db != NULL) {
            *snr_db = (int8_t)pkt_status[3] / 4;
        }
    }

    *out_len = len;
    sx1281_clear_irq();
    sx1281_set_rx();
    return true;
}
