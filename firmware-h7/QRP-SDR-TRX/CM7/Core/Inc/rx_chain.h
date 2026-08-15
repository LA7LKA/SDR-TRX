#ifndef RX_CHAIN_H
#define RX_CHAIN_H

#include <stdint.h>

/*
 * Per-mode RX demodulation, ported from firmware/Core/Src/main.c (where
 * these lived inline with mode state and filter setup). Each function
 * reads one block of raw 12-bit ADC samples (in) and writes one block of
 * 12-bit DAC samples (out), n samples each - n and the buffer depth vary
 * per mode (see modes.h's BLOCK_SIZE_*), not fixed here.
 */

void ssb_process_block(const uint16_t *in, uint16_t *out, int n);
void nbfm_process_block(const uint16_t *in, uint16_t *out, int n);
void am_process_block(const uint16_t *in, uint16_t *out, int n);

/*
 * FreeDV 2400A/2400B receive - rides inside an ordinary FM channel, so this
 * is the FM path, not the SSB one. Deliberately no post-demod audio LPF:
 * the voice filter would cut the 4800 Hz tone and the modem would never
 * sync.
 */
void freedv2400b_process_block(const uint16_t *in, uint16_t *out, int n);

/* RX dispatch: routes to exactly one of the above by MODE. */
void process_block(const uint16_t *in, uint16_t *out, int n);

#endif
