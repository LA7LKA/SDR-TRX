#include "cw_paddle.h"
#include "stm32f7xx_hal.h"

/* See cw_paddle.h - placeholder pins, confirm against real wiring. */
#define CW_PADDLE_DIT_PORT   GPIOG
#define CW_PADDLE_DIT_PIN    GPIO_PIN_9
#define CW_PADDLE_DAH_PORT   GPIOG
#define CW_PADDLE_DAH_PIN    GPIO_PIN_12

void cw_paddle_gpio_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();

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
