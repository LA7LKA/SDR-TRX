#ifndef BUTTONS_H
#define BUTTONS_H
#include <stdint.h>

/* Nucleo-H755ZI-Q ST Zio header, wired per UM2408 Table 17: D2,D3,D4,D5,
   D9,D10,D11,D14,D15,A0 - avoiding D0/D1 (this board's I2C1 for the OLED),
   D6 (USB_OTG_FS_SOF), D7/D8 (reserved for a planned UART-to-Bluetooth
   link), and D12/D13 (ADC2 mic-in / DAC1_CH2). See buttons.c. */
#define BUTTON_COUNT 10

void buttons_init(void);

/* Bit (n-1) set = button n is currently pressed. Switches to GND with
   internal pull-ups, so this already accounts for the active-low wiring -
   a set bit here means pressed, not "pin low". */
uint16_t buttons_read(void);

#endif
