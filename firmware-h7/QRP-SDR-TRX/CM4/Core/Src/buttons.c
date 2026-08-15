#include "buttons.h"
#include "stm32h7xx_hal.h"

typedef struct { GPIO_TypeDef *port; uint16_t pin; } btn_pin_t;

/* D2,D3,D4,D5,D9,D10,D11,D14,D15,A0 in wiring order - see buttons.h for
   why D0/D1/D6/D7/D8/D12/D13 are skipped. Mapped from UM2408 Table 17
   (NUCLEO-H745ZI-Q and NUCLEO-H755ZI-Q pin assignments), not carried over
   from F746 - this board's Arduino-header-to-MCU-pin map is different. */
static const btn_pin_t btn_pins[BUTTON_COUNT] = {
    { GPIOG, GPIO_PIN_14 },  /* 1:  D2  */
    { GPIOE, GPIO_PIN_13 },  /* 2:  D3  */
    { GPIOE, GPIO_PIN_14 },  /* 3:  D4  */
    { GPIOE, GPIO_PIN_11 },  /* 4:  D5  */
    { GPIOD, GPIO_PIN_15 },  /* 5:  D9  */
    { GPIOD, GPIO_PIN_14 },  /* 6:  D10 */
    { GPIOB, GPIO_PIN_5  },  /* 7:  D11 */
    { GPIOB, GPIO_PIN_9  },  /* 8:  D14 */
    { GPIOB, GPIO_PIN_8  },  /* 9:  D15 */
    { GPIOA, GPIO_PIN_3  },  /* 10: A0  */
};

void buttons_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

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
