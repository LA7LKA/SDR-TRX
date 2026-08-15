#ifndef HMI_H
#define HMI_H
#include "stm32h7xx_hal.h"

/*
 * Front-panel controller: owns the encoder, the 10 buttons, the OLED, and
 * PTT. Ported from firmware/Core/Src/hmi.c - the biggest structural change
 * is that on F746 this called radio_set_mode()/radio_tx_on()/cw_set_pitch()
 * etc. directly (same core, same address space); here those functions live
 * on CM7, so every one of those calls becomes "update local desired state,
 * push it through the IPC mailbox" instead - see hmi.c.
 */

void hmi_init(I2C_HandleTypeDef *oled_i2c);

/* Call every main-loop iteration. */
void hmi_poll(void);

/* console.c's "mode" command - see hmi.c for why this goes through the
   same desired_mode/ipc_push() path as BTN_MODE rather than writing the
   IPC command directly. */
void hmi_set_mode(int32_t mode);

#endif
