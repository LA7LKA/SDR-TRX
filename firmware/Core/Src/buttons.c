#include "buttons.h"
#include "stm32f7xx_hal.h"

typedef struct { GPIO_TypeDef *port; uint16_t pin; } btn_pin_t;

/* D2..D10 + D12, in wiring order. See buttons.h for why D11 is skipped. */
static const btn_pin_t btn_pins[BUTTON_COUNT] = {
    { GPIOF, GPIO_PIN_15 },  /* 1:  D2  */
    { GPIOE, GPIO_PIN_13 },  /* 2:  D3  */
    { GPIOF, GPIO_PIN_14 },  /* 3:  D4  */
    { GPIOE, GPIO_PIN_11 },  /* 4:  D5  */
    { GPIOE, GPIO_PIN_9  },  /* 5:  D6  */
    { GPIOF, GPIO_PIN_13 },  /* 6:  D7  */
    { GPIOF, GPIO_PIN_12 },  /* 7:  D8  */
    { GPIOD, GPIO_PIN_15 },  /* 8:  D9  */
    { GPIOD, GPIO_PIN_14 },  /* 9:  D10 */
    { GPIOA, GPIO_PIN_6  },  /* 10: D12 */
};

void buttons_init(void)
{
    /* GPIOA/GPIOD are already clocked elsewhere in this firmware; harmless
       to enable again. GPIOE/GPIOF are new. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        gpio.Pin = btn_pins[i].pin;
        HAL_GPIO_Init(btn_pins[i].port, &gpio);
    }
}

uint16_t buttons_read(void)
{
    uint16_t mask = 0;
    for (int i = 0; i < BUTTON_COUNT; i++)
        if (HAL_GPIO_ReadPin(btn_pins[i].port, btn_pins[i].pin) == GPIO_PIN_RESET)
            mask |= (uint16_t)(1u << i);
    return mask;
}
