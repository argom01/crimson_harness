#include "sx1281.h"
#include "app_error.h"
#include "config.h"
#include "spi_bus.h"
#include "main.h"

/* Semtech SX1280/SX1281 command opcodes */
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
#define RADIO_WRITE_REGISTER      0x18U
#define RADIO_READ_REGISTER       0x19U
#define RADIO_WRITE_BUFFER        0x1AU
#define RADIO_READ_BUFFER         0x1BU
#define RADIO_SET_RX              0x82U
#define RADIO_SET_TX              0x83U

#define PACKET_TYPE_LORA          0x01U
#define STDBY_XOSC                0x01U
#define LORA_BW_1600              0x0AU
#define LORA_SF5                  0x50U   /* SX128x codes SF in the high nibble */
#define LORA_CR_4_5               0x01U
#define IRQ_RX_DONE               0x0002U
#define IRQ_TX_DONE               0x0001U

#define SYNC_WORD_REG             0x0944U

/* BUSY is normally released within tens of microseconds; a stuck pin means
   the radio is wedged or absent and must not stall the control loop. */
#define BUSY_TIMEOUT_MS           5U

static uint8_t consecutive_errors;

static void comm_fail(app_error_t err)
{
    if (consecutive_errors < UINT8_MAX) {
        consecutive_errors++;
    }
    app_error_report(err);
}

static void comm_good(void)
{
    consecutive_errors = 0U;
}

bool sx1281_healthy(void)
{
    return consecutive_errors < RADIO_MAX_CONSECUTIVE_ERRORS;
}

static bool wait_not_busy(void)
{
    const uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(RF24_BUSY_GPIO_Port, RF24_BUSY_Pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) > BUSY_TIMEOUT_MS) {
            comm_fail(APP_ERR_RF24_BUSY);
            return false;
        }
    }
    return true;
}

static void reset_radio(void)
{
    HAL_GPIO_WritePin(RF24_RST_GPIO_Port, RF24_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RF24_RST_GPIO_Port, RF24_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

static bool send_command(uint8_t cmd, const uint8_t *params, uint16_t param_len)
{
    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    HAL_StatusTypeDef st = spi_bus_write(&cmd, 1U);
    if (st == HAL_OK && param_len > 0U && params != NULL) {
        st = spi_bus_write(params, param_len);
    }
    spi_bus_deselect(SPI_DEV_RF24);

    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();
    return true;
}

static bool get_status(uint8_t cmd, uint8_t *resp, uint16_t resp_len)
{
    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    HAL_StatusTypeDef st = spi_bus_write(&cmd, 1U);
    if (st == HAL_OK && resp_len > 0U && resp != NULL) {
        st = spi_bus_read(resp, resp_len);
    }
    spi_bus_deselect(SPI_DEV_RF24);

    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();
    return true;
}

static bool write_register(uint16_t addr, const uint8_t *data, uint16_t len)
{
    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    uint8_t hdr[3] = {RADIO_WRITE_REGISTER, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFFU)};
    HAL_StatusTypeDef st = spi_bus_write(hdr, 3U);
    if (st == HAL_OK) {
        st = spi_bus_write(data, len);
    }
    spi_bus_deselect(SPI_DEV_RF24);

    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();
    return true;
}

/* SX128x read register frame: opcode -> addr[2] -> NOP -> data */
static bool read_register(uint16_t addr, uint8_t *data, uint16_t len)
{
    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    uint8_t hdr[4] = {RADIO_READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFFU), 0x00U};
    HAL_StatusTypeDef st = spi_bus_write(hdr, 4U);
    if (st == HAL_OK) {
        st = spi_bus_read(data, len);
    }
    spi_bus_deselect(SPI_DEV_RF24);

    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();
    return true;
}

static bool set_frequency(uint32_t freq_hz)
{
    uint8_t buf[3];
    /* PLL step = 52 MHz / 2^18 */
    uint32_t reg = (uint32_t)((double)freq_hz / 52000000.0 * (double)(1UL << 18));
    buf[0] = (uint8_t)((reg >> 16) & 0xFFU);
    buf[1] = (uint8_t)((reg >> 8) & 0xFFU);
    buf[2] = (uint8_t)(reg & 0xFFU);
    return send_command(RADIO_SET_RFFREQUENCY, buf, 3U);
}

static bool configure_lora(void)
{
    bool ok = true;

    uint8_t pkt_type = PACKET_TYPE_LORA;
    ok = ok && send_command(RADIO_SET_PACKETTYPE, &pkt_type, 1U);

    uint8_t mod[] = {LORA_SF5, LORA_BW_1600, LORA_CR_4_5};
    ok = ok && send_command(RADIO_SET_MODULATIONPARAMS, mod, 3U);

    /* Datasheet: after SetModulationParams write reg 0x0925 (0x1E for SF5/SF6)
       and 0x01 to the Frequency Error Compensation register 0x093C */
    uint8_t sf_cfg = 0x1EU;
    ok = ok && write_register(0x0925U, &sf_cfg, 1U);
    uint8_t fec = 0x01U;
    ok = ok && write_register(0x093CU, &fec, 1U);

    /* preamble 12 symbols (mant 3 * 2^exp 2; both fields must be in [1:15]),
       explicit header, max payload 255, CRC on (0x20), standard IQ (0x40) */
    uint8_t pkt[] = {0x23U, 0x00U, 0xFFU, 0x20U, 0x40U, 0x00U, 0x00U};
    ok = ok && send_command(RADIO_SET_PACKETPARAMS, pkt, 7U);

    uint8_t base[] = {0x00U, 0x00U};
    ok = ok && send_command(RADIO_SET_BUFFERBASEADDRESS, base, 2U);

    /* LoRa sync word register 0x0944/0x0945, nibble-coded with 0x44 control bits */
    uint8_t sync[2] = {
        (uint8_t)((RF24_SYNC_WORD & 0xF0U) | 0x04U),
        (uint8_t)(((RF24_SYNC_WORD & 0x0FU) << 4) | 0x04U),
    };
    ok = ok && write_register(SYNC_WORD_REG, sync, 2U);

    /* Verify a register readback so a dead or absent radio fails init
       instead of silently accepting writes into nowhere. */
    uint8_t sync_check[2] = {0U, 0U};
    ok = ok && read_register(SYNC_WORD_REG, sync_check, 2U);
    if (ok && (sync_check[0] != sync[0] || sync_check[1] != sync[1])) {
        ok = false;
    }

    /* IrqMask = TxDone | RxDone */
    uint8_t irq[] = {0x00U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    ok = ok && send_command(RADIO_SET_DIOIRQPARAMS, irq, 8U);

    return ok;
}

bool sx1281_init(void)
{
    reset_radio();
    if (!wait_not_busy()) {
        return false;
    }

    bool ok = true;

    uint8_t standby = STDBY_XOSC;
    ok = ok && send_command(RADIO_SET_STANDBY, &standby, 1U);
    ok = ok && configure_lora();
    ok = ok && set_frequency(RF24_FREQUENCY_HZ);

    uint8_t tx_params[] = {RF24_TX_POWER, 0xE0U}; /* 20 us ramp */
    ok = ok && send_command(RADIO_SET_TXPARAMS, tx_params, 2U);

    ok = ok && sx1281_clear_irq();
    ok = ok && sx1281_set_rx();
    return ok;
}

bool sx1281_set_rx(void)
{
    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    return send_command(RADIO_SET_RX, timeout, 3U);
}

bool sx1281_set_tx(const uint8_t *payload, uint8_t len)
{
    if (payload == NULL || len == 0U) {
        return false;
    }

    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    uint8_t cmd = RADIO_WRITE_BUFFER;
    uint8_t offset = 0x00U;
    HAL_StatusTypeDef st = spi_bus_write(&cmd, 1U);
    if (st == HAL_OK) {
        st = spi_bus_write(&offset, 1U);
    }
    if (st == HAL_OK) {
        st = spi_bus_write(payload, len);
    }
    spi_bus_deselect(SPI_DEV_RF24);
    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();

    uint8_t timeout[] = {0x00U, 0x00U, 0x00U};
    return send_command(RADIO_SET_TX, timeout, 3U);
}

bool sx1281_irq_rx_done(void)
{
    uint8_t status[3];
    if (!get_status(RADIO_GET_IRQSTATUS, status, 3U)) {
        return false;
    }
    uint16_t irq = (uint16_t)((status[1] << 8) | status[2]);
    return (irq & IRQ_RX_DONE) != 0U;
}

bool sx1281_clear_irq(void)
{
    uint8_t clr[] = {0xFFU, 0xFFU};
    return send_command(RADIO_CLR_IRQSTATUS, clr, 2U);
}

bool sx1281_read_packet(uint8_t *payload, uint8_t max_len, uint8_t *out_len, int16_t *rssi_dbm, int8_t *snr_db)
{
    if (payload == NULL || out_len == NULL) {
        return false;
    }

    uint8_t rx_status[3];
    if (!get_status(RADIO_GET_RXBUFFERSTATUS, rx_status, 3U)) {
        return false;
    }
    uint8_t len = rx_status[1];
    if (len == 0U || len > max_len) {
        (void)sx1281_clear_irq();
        (void)sx1281_set_rx();
        return false;
    }

    /* ReadBuffer frame: opcode -> offset -> NOP -> data */
    if (!wait_not_busy()) {
        return false;
    }
    spi_bus_select(SPI_DEV_RF24);
    uint8_t tx[3U + 255U];
    uint8_t rx[3U + 255U];
    tx[0] = RADIO_READ_BUFFER;
    tx[1] = rx_status[2];
    tx[2] = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        tx[3U + i] = 0U;
    }
    HAL_StatusTypeDef st = spi_bus_transfer(tx, rx, (uint16_t)(3U + len));
    spi_bus_deselect(SPI_DEV_RF24);
    if (st != HAL_OK) {
        comm_fail(APP_ERR_SPI);
        return false;
    }
    comm_good();
    for (uint8_t i = 0U; i < len; i++) {
        payload[i] = rx[3U + i];
    }

    if (rssi_dbm != NULL || snr_db != NULL) {
        /* pkt_status[0] = chip status, [1] = rssiSync, [2] = snr */
        uint8_t pkt_status[3];
        if (get_status(RADIO_GET_PACKETSTATUS, pkt_status, 3U)) {
            if (rssi_dbm != NULL) {
                *rssi_dbm = (int16_t)(-(int16_t)pkt_status[1] / 2);
            }
            if (snr_db != NULL) {
                *snr_db = (int8_t)pkt_status[2] / 4;
            }
        } else {
            if (rssi_dbm != NULL) {
                *rssi_dbm = -127;
            }
            if (snr_db != NULL) {
                *snr_db = 0;
            }
        }
    }

    *out_len = len;
    (void)sx1281_clear_irq();
    (void)sx1281_set_rx();
    return true;
}
