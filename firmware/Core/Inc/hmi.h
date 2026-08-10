#ifndef HMI_H
#define HMI_H
#include "stm32f7xx_hal.h"

/*
 * Front-panel controller for the UI test board: owns the encoder, the 10
 * buttons, the OLED, and PTT (the Nucleo's onboard USER_Btn on this test
 * setup - a real front panel would have its own PTT line). See hmi.c for
 * what's real (mode, PTT, mic gain) versus a UI skeleton with no hardware
 * behind it yet (freq, band, RIT, CW pitch, volume).
 */

void hmi_init(I2C_HandleTypeDef *oled_i2c);

/* Call every main-loop iteration. */
void hmi_poll(void);

/* CAT-control accessors for the UI-only vfo_freq_hz placeholder (see hmi.c
   for why it's cosmetic - no real VFO/LO exists yet). hmi_set_vfo_freq()
   redraws immediately since it's called from outside hmi_poll()'s own
   throttled redraw path. */
uint32_t hmi_get_vfo_freq(void);
void hmi_set_vfo_freq(uint32_t freq_hz);

#endif
