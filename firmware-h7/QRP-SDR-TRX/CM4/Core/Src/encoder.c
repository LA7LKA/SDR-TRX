#include "encoder.h"
#include "stm32h7xx_hal.h"

/* A1 (PC0) and A3 (PB1) - different ports on this board, unlike F746
   where both phases happened to land on GPIOB. See encoder.h. */
#define ENC_A_PORT GPIOC
#define ENC_A_PIN  GPIO_PIN_0
#define ENC_B_PORT GPIOB
#define ENC_B_PIN  GPIO_PIN_1

static int32_t count = 0;
static uint8_t prev_state = 0;

/*
 * Standard quadrature decode table, indexed by (prev_state << 2 | new_state)
 * where state = (A << 1 | B). Invalid/bounce transitions map to 0, a valid
 * one-step transition maps to +-1. This encoder has no detents, so every
 * valid quadrature step counts - there's no "one detent = four transitions"
 * convention to divide by here.
 */
static const int8_t qtable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t read_state(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(ENC_A_PORT, ENC_A_PIN) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(ENC_B_PORT, ENC_B_PIN) == GPIO_PIN_SET) ? 1 : 0;
    return (uint8_t)((a << 1) | b);
}

void encoder_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    gpio.Pin = ENC_A_PIN;
    HAL_GPIO_Init(ENC_A_PORT, &gpio);

    gpio.Pin = ENC_B_PIN;
    HAL_GPIO_Init(ENC_B_PORT, &gpio);

    prev_state = read_state();
}

int32_t encoder_poll(void)
{
    uint8_t now = read_state();
    count += qtable[(prev_state << 2) | now];
    prev_state = now;
    return count;
}
