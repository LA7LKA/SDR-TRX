#ifndef CW_PADDLE_H
#define CW_PADDLE_H
#include <stdint.h>

/*
 * Raw GPIO for a CW key/paddle input - hardware access only, no keyer
 * logic (that's cw_keyer_* in main.c, which drives the real-time state
 * machine and calls these). Convention: a single stereo jack, tip=dit
 * (also the only contact a straight key/mono plug uses), ring=dah,
 * sleeve=common ground - the same convention real transceivers use so
 * one jack serves both a straight key and an iambic paddle, selected by
 * the operator via a menu setting rather than by which jack is used.
 *
 * PLACEHOLDER PIN ASSIGNMENT - no key/paddle hardware exists yet
 * (Oystein hasn't bought/built one). PG9/PG12 were picked because
 * nothing else in the firmware currently claims them, not because
 * they're confirmed free on the physical Nucleo-F746ZG headers - swap
 * cw_paddle.c's #defines once real wiring is decided.
 */

void    cw_paddle_gpio_init(void);
uint8_t cw_paddle_dit_read(void);   /* 1 = pressed/down (straight key uses only this) */
uint8_t cw_paddle_dah_read(void);   /* 1 = pressed/down (paddle only) */

#endif
