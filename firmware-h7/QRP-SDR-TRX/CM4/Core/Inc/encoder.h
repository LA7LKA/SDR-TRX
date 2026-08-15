#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

/* Quadrature encoder on Nucleo-H755ZI-Q Arduino header pins A1 (PC0, A)
   and A3 (PB1, B) - per UM2408 Table 17, internal pull-ups, wire the
   encoder's common pin to GND. ALPS EC12E24104A6 has no detents and no
   integrated switch, so this is just the two phase inputs (same part and
   same reasoning as F746; only the MCU pins differ). */

void encoder_init(void);

/* Call every main-loop iteration. Returns the running count so far; only
   changes on a valid quadrature step, so no separate debounce is needed. */
int32_t encoder_poll(void);

#endif
