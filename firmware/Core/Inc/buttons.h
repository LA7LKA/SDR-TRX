#ifndef BUTTONS_H
#define BUTTONS_H
#include <stdint.h>

/* UI test board, wired 2026-08-03: D2-D10 + D12 (D11 is reserved, shared
   with Ethernet on this Nucleo). Numbered 1-10 in wiring order. */
#define BUTTON_COUNT 10

void buttons_init(void);

/* Bit (n-1) set = button n is currently pressed. Switches to GND with
   internal pull-ups, so this already accounts for the active-low wiring -
   a set bit here means pressed, not "pin low". */
uint16_t buttons_read(void);

#endif
