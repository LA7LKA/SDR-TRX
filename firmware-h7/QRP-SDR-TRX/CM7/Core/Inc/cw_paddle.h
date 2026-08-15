#ifndef CW_PADDLE_H
#define CW_PADDLE_H
#include <stdint.h>

/*
 * Raw GPIO for a CW key/paddle input - hardware access only, no keyer
 * logic (that's cw_keyer_* in tx_chain.c, which drives the real-time
 * state machine and calls these). Convention: a single stereo jack,
 * tip=dit (also the only contact a straight key/mono plug uses),
 * ring=dah, sleeve=common ground - the same convention real transceivers
 * use so one jack serves both a straight key and an iambic paddle,
 * selected by the operator via a menu setting rather than by which jack
 * is used.
 *
 * Lives on CM7, not CM4 with the rest of the HMI-family input code:
 * cw_keyer_tick() (tx_chain.c) reads these once per audio sample at
 * 48 kHz from inside the TX signal chain - routing that through the
 * CM4<->CM7 IPC mailbox would mean 48,000 cross-core messages a second
 * for a 2-bit paddle state, which isn't workable. So the paddle GPIOs are
 * read directly on the core that actually consumes them at that rate.
 *
 * PIN ASSIGNMENT: PB10 (dit/straight-key)/PB11 (dah), reused from F746 -
 * free on this board's .ioc too. Assign to the CortexM7 context in
 * CubeMX (not CortexM4), matching where this file now lives.
 */

void    cw_paddle_gpio_init(void);
uint8_t cw_paddle_dit_read(void);   /* 1 = pressed/down (straight key uses only this) */
uint8_t cw_paddle_dah_read(void);   /* 1 = pressed/down (paddle only) */

#endif
