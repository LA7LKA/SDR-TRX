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
 * PIN ASSIGNMENT: PB10 (dit/straight-key)/PB11 (dah) - no key/paddle
 * hardware exists yet (Oystein hasn't bought/built one), but these two
 * were deliberately picked (2026-08-11) over other free-looking pins to
 * leave PG9/PG12 open for a planned UART-to-Bluetooth-module link. Swap
 * cw_paddle.c's #defines if the wiring plan changes again.
 */

void    cw_paddle_gpio_init(void);
uint8_t cw_paddle_dit_read(void);   /* 1 = pressed/down (straight key uses only this) */
uint8_t cw_paddle_dah_read(void);   /* 1 = pressed/down (paddle only) */

#endif
