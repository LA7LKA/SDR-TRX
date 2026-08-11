#include "cw_paddle.h"
#include "stm32f7xx_hal.h"

/* See cw_paddle.h - PB10/PB11 chosen 2026-08-11 specifically to leave
   PG9/PG12 free for a planned UART-to-Bluetooth-module link. */
#define CW_PADDLE_DIT_PORT   GPIOB
#define CW_PADDLE_DIT_PIN    GPIO_PIN_10
#define CW_PADDLE_DAH_PORT   GPIOB
#define CW_PADDLE_DAH_PIN    GPIO_PIN_11

void cw_paddle_gpio_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = CW_PADDLE_DIT_PIN | CW_PADDLE_DAH_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CW_PADDLE_DIT_PORT, &gpio);   /* same port for both pins here */
}

uint8_t cw_paddle_dit_read(void)
{
    return (HAL_GPIO_ReadPin(CW_PADDLE_DIT_PORT, CW_PADDLE_DIT_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t cw_paddle_dah_read(void)
{
    return (HAL_GPIO_ReadPin(CW_PADDLE_DAH_PORT, CW_PADDLE_DAH_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}
