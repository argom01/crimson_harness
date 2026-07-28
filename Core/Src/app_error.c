#include "app_error.h"
#include "main.h"

/* BKP_DR1 layout: high byte = magic marking a recorded fatal error,
   low byte = app_error_t code. Backup domain survives NVIC_SystemReset. */
#define FATAL_MAGIC_MASK  0xFF00U
#define FATAL_MAGIC       0xE500U

static uint16_t error_counts[APP_ERR_COUNT];
static app_error_t last_error = APP_ERR_NONE;
static uint32_t last_error_ms;

static void bkp_access_enable(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

void app_error_report(app_error_t err)
{
    if (err >= APP_ERR_COUNT) {
        return;
    }
    if (error_counts[err] < UINT16_MAX) {
        error_counts[err]++;
    }
    last_error = err;
    last_error_ms = HAL_GetTick();
}

void app_error_fatal(app_error_t err)
{
    __disable_irq();
    bkp_access_enable();
    BKP->DR1 = (uint16_t)(FATAL_MAGIC | ((uint16_t)err & 0xFFU));
    NVIC_SystemReset();
    while (1) {
    }
}

uint16_t app_error_count(app_error_t err)
{
    return (err < APP_ERR_COUNT) ? error_counts[err] : 0U;
}

app_error_t app_error_last(void)
{
    return last_error;
}

uint32_t app_error_last_ms(void)
{
    return last_error_ms;
}

app_error_t app_error_boot_fault(void)
{
    bkp_access_enable();
    const uint16_t raw = (uint16_t)BKP->DR1;
    if ((raw & FATAL_MAGIC_MASK) != FATAL_MAGIC) {
        return APP_ERR_NONE;
    }
    BKP->DR1 = 0U;
    return (app_error_t)(raw & 0xFFU);
}
