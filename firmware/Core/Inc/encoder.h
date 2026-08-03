#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

/* Quadrature encoder on PB4 (A) / PB5 (B), internal pull-ups - wire the
   encoder's common pin to GND. ALPS EC12E24104A6 has no detents and no
   integrated switch, so this is just the two phase inputs. */

void encoder_init(void);

/* Call every main-loop iteration. Returns the running count so far; only
   changes on a valid quadrature step, so no separate debounce is needed. */
int32_t encoder_poll(void);

#endif
