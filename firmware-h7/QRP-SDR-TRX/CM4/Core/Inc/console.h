#ifndef CONSOLE_H
#define CONSOLE_H
#include "stm32h7xx_hal.h"

/*
 * Interactive debug console over USART3 (the ST-LINK VCP on this board, see
 * main.c) - "help"/"status"/"mode <name>" for now. Lives on CM4 rather than
 * CM7: CM4 owns the control plane (HMI/IPC) and isn't running the
 * time-critical DSP loop or USB Audio class handling.
 */

/* huart must already be HAL_UART_Init()'d (main.c does this before calling
   in). Enables USART3's RX interrupt and arms the first single-byte
   receive. */
void console_init(UART_HandleTypeDef *huart);

/* Call every main-loop iteration - cheap, does nothing until a full line
   has arrived. */
void console_poll(void);

#endif
