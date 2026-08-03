#ifndef OLED_H
#define OLED_H
#include "stm32f7xx_hal.h"

/*
 * Minimal SSD1306 driver, 128x64 over I2C1 (PB8/PB9, address 0x3C).
 * Uppercase-only 5x7 font covering space, 0-9, and the letters this
 * project's bring-up tests actually print (CDEHLNORTUW), plus ':' and '-'.
 * Characters outside that set are skipped rather than guessed at.
 */

void oled_init(I2C_HandleTypeDef *hi2c);
void oled_clear(void);

/* col in pixels (0-127), page in 8-pixel rows (0-7). Each glyph is 5 px
   wide plus 1 px spacing, so page text wraps at column 128 silently. */
void oled_draw_text(uint8_t col, uint8_t page, const char *s);

void oled_display(void);

#endif
