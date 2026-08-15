#ifndef TX_CHAIN_H
#define TX_CHAIN_H

#include <stdint.h>

/*
 * Per-mode TX modulation, ported from firmware/Core/Src/main.c - including
 * the CW keyer state machine, which F746's original had inline in main.c
 * but which fits naturally here alongside CW's own TX processing function,
 * the same way SSB/AM/NBFM's TX functions sit next to their own support
 * code. (Corrected from this port's original plan, which guessed the
 * keyer would stay in main.c before the actual code was read.)
 */

void ssb_tx_process_block(const uint16_t *in, uint16_t *out, int n);
void nbfm_tx_process_block(const uint16_t *in, uint16_t *out, int n);
void am_tx_process_block(const uint16_t *in, uint16_t *out, int n);

/* CW: message-beacon TX (looping, console/CAT "tx") and live key/paddle TX
   (cw_keyer_poll(), main-loop rate) both feed cw_tx_process_block() - see
   tx_chain.c for how the two sources are picked. side_out is the sidetone
   buffer; only CW drives it. */
void cw_tx_restart(void);
void cw_tx_process_block(uint16_t *out, uint16_t *side_out, int n);

/* Main-loop-rate (not sample-rate) poll for the physical key/paddle -
   starts/stops its own TX session the same coarse way PTT does for voice
   modes. Call every main loop iteration. */
void cw_keyer_poll(void);

/* CW message (beacon text), WPM, and keyer type - console/CAT/HMI-settable
   (HMI via the IPC mailbox on H7, since hmi.c is on CM4). */
void        cw_set_message(const char *text);
const char *cw_get_message(void);
void        cw_set_wpm(int wpm);
int         cw_get_wpm(void);
int         cw_keyer_type_get(void);       /* 0=straight, 1=iambic A, 2=iambic B */
void        cw_keyer_type_set(int t);
const char *cw_keyer_type_name(void);

/* FreeDV TX: feed mic audio in (every block, cheap - decimate only) and
   pull modulated IF out (every block, cheap - no encode). The heavy
   encode itself is freedv_chain_encode() (freedv_chain.h), called from
   the main loop ahead of the DAC fill, not from either of these. */
void freedv_tx_feed(const uint16_t *in, int n);
void freedv_tx_produce(uint16_t *out, int n);

/* Transmit dispatch: routes to exactly one of the above by MODE. side_out
   is the CW sidetone buffer - only CW writes it. */
void tx_process_block(const uint16_t *in, uint16_t *out, uint16_t *side_out, int n);
int  tx_supported(int mode);

/* Set from the console/CAT "tone" command - substitutes a clean internal
   tone for the mic on AM TX, to isolate the modulator from mic/ADC issues
   when bench testing. */
extern volatile int am_testtone;

#endif
