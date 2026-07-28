#include "uart_link.h"
#include "app_error.h"
#include "main.h"

/* DMA-backed FC UART link on the CubeMX-generated HAL handles
   (hdma_usart1_rx circular, hdma_usart1_tx normal, USART1 + DMA IRQs
   enabled). All HAL calls happen in main-loop context; the HAL ISRs only
   complete transfers, so there is no shared bookkeeping to race on.

   512 bytes of RX buffer is ~44 ms of headroom at 115200 baud, far more
   than one loop pass. */
#define UART_RX_BUF_LEN 512U
#define UART_TX_BUF_LEN 512U

/* A full TX buffer drains in ~45 ms at 115200 baud. If the handle is still
   BUSY_TX long after that, a completion interrupt was lost or the
   peripheral wedged: abort so the stream can recover. */
#define UART_TX_STUCK_MS 100U

static uint8_t rx_buf[UART_RX_BUF_LEN];
static uint16_t rx_tail;

static uint8_t tx_buf[UART_TX_BUF_LEN];
static uint16_t tx_head;     /* start of pending data (in-flight first) */
static uint16_t tx_count;    /* pending bytes, including in-flight chunk */
static uint16_t tx_inflight; /* bytes handed to the current DMA transfer */
static uint32_t tx_start_ms;

static void rx_arm(void)
{
    rx_tail = 0U;
    if (HAL_UART_Receive_DMA(&huart1, rx_buf, UART_RX_BUF_LEN) != HAL_OK) {
        app_error_report(APP_ERR_UART_RX);
    }
}

void uart_link_init(void)
{
    tx_head = 0U;
    tx_count = 0U;
    tx_inflight = 0U;
    rx_arm();
}

bool uart_link_read_byte(uint8_t *byte)
{
    const uint16_t head =
        (uint16_t)(UART_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (head == rx_tail) {
        return false;
    }
    *byte = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUF_LEN);
    return true;
}

/* Reclaim the completed chunk and start the next contiguous one. */
static void tx_kick(void)
{
    if (huart1.gState != HAL_UART_STATE_READY) {
        if (tx_inflight != 0U && (HAL_GetTick() - tx_start_ms) > UART_TX_STUCK_MS) {
            /* Completion never arrived; abort, drop the chunk, resync */
            (void)HAL_UART_AbortTransmit(&huart1);
            app_error_report(APP_ERR_UART_TX);
            tx_head = (uint16_t)((tx_head + tx_inflight) % UART_TX_BUF_LEN);
            tx_count = (uint16_t)(tx_count - tx_inflight);
            tx_inflight = 0U;
        }
        return;
    }

    if (tx_inflight != 0U) {
        tx_head = (uint16_t)((tx_head + tx_inflight) % UART_TX_BUF_LEN);
        tx_count = (uint16_t)(tx_count - tx_inflight);
        tx_inflight = 0U;
    }

    if (tx_count == 0U) {
        return;
    }

    uint16_t chunk = (uint16_t)(UART_TX_BUF_LEN - tx_head);
    if (chunk > tx_count) {
        chunk = tx_count;
    }
    if (HAL_UART_Transmit_DMA(&huart1, &tx_buf[tx_head], chunk) == HAL_OK) {
        tx_inflight = chunk;
        tx_start_ms = HAL_GetTick();
    }
}

bool uart_link_write(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > UART_TX_BUF_LEN) {
        return false;
    }

    tx_kick(); /* reclaim space finished since the last poll */

    if ((uint16_t)(UART_TX_BUF_LEN - tx_count) < len) {
        app_error_report(APP_ERR_UART_TX);
        return false;
    }

    uint16_t tail = (uint16_t)((tx_head + tx_count) % UART_TX_BUF_LEN);
    for (uint16_t i = 0U; i < len; i++) {
        tx_buf[tail] = data[i];
        tail = (uint16_t)((tail + 1U) % UART_TX_BUF_LEN);
    }
    tx_count = (uint16_t)(tx_count + len);

    tx_kick();
    return true;
}

void uart_link_poll(void)
{
    /* On an overrun the HAL aborts circular RX (RxState back to READY) and
       stops receiving. Re-arm so a noise burst can never silence the FC
       link permanently. */
    if (huart1.RxState == HAL_UART_STATE_READY) {
        app_error_report(APP_ERR_UART_RX);
        rx_arm();
    }

    tx_kick();
}
