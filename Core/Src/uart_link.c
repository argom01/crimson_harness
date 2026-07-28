#include "uart_link.h"
#include "app_error.h"
#include "main.h"

/* USART1 DMA mapping on STM32F103: RX = DMA1 channel 5, TX = DMA1 channel 4.
   Both directions are driven at register level and polled from the main
   loop: no interrupts, no HAL UART state machine, nothing that can wedge.

   512 bytes of RX buffer is ~44 ms of headroom at 115200 baud, far more
   than one loop pass. */
#define UART_RX_BUF_LEN 512U
#define UART_TX_BUF_LEN 512U

static uint8_t rx_buf[UART_RX_BUF_LEN];
static uint16_t rx_tail;

static uint8_t tx_buf[UART_TX_BUF_LEN];
static uint16_t tx_head;     /* start of pending data (in-flight first) */
static uint16_t tx_count;    /* pending bytes, including in-flight chunk */
static uint16_t tx_inflight; /* bytes handed to the current DMA transfer */

void uart_link_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* RX: circular buffer, DMA runs forever, consumer chases CNDTR */
    DMA1_Channel5->CCR = 0U;
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)rx_buf;
    DMA1_Channel5->CNDTR = UART_RX_BUF_LEN;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;

    /* TX: one-shot transfers restarted chunk by chunk from the ring */
    DMA1_Channel4->CCR = 0U;
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;

    SET_BIT(USART1->CR3, USART_CR3_DMAR | USART_CR3_DMAT);

    rx_tail = 0U;
    tx_head = 0U;
    tx_count = 0U;
    tx_inflight = 0U;
}

bool uart_link_read_byte(uint8_t *byte)
{
    const uint16_t head = (uint16_t)(UART_RX_BUF_LEN - DMA1_Channel5->CNDTR);
    if (head == rx_tail) {
        return false;
    }
    *byte = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUF_LEN);
    return true;
}

/* Reclaim finished DMA space and start the next contiguous chunk. */
static void tx_kick(void)
{
    if ((DMA1_Channel4->CCR & DMA_CCR_EN) != 0U) {
        if (DMA1_Channel4->CNDTR != 0U) {
            return; /* previous chunk still in flight */
        }
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF4;
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
    tx_inflight = chunk;
    DMA1_Channel4->CMAR = (uint32_t)&tx_buf[tx_head];
    DMA1_Channel4->CNDTR = chunk;
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
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
    /* Noise/overrun flags (FE/NE/PE/ORE) are cleared by an SR read followed
       by a DR read; the DMA's own DR access completes the sequence. Left
       uncleared they would not stop DMA, but clear them for hygiene. */
    const uint32_t sr = USART1->SR;
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
        (void)sr;
    }

    tx_kick();
}
