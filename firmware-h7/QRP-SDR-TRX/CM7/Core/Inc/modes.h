#ifndef MODES_H
#define MODES_H

#include <stdint.h>

/*
 * Block size is per mode, so buffers (dsp_scratch, adc_buffer/dac_buffer -
 * see main.c) are allocated for the largest case and the DMA is restarted
 * with the size the current mode wants. Shared here (not just in modes.c)
 * since filters.c/rx_chain.c/tx_chain.c/main.c all need these to size their
 * own buffers - one definition instead of four copies risking drift.
 *
 * FreeDV: half a buffer is 1920 samples = 40 ms, and 1920/6 = 320, which is
 * exactly one FDMDV frame. That makes every block cost the same instead of
 * alternating cheap/expensive, which is what it takes to stay real time.
 *
 * FreeDV 2400B: 96 bits at 4800 baud, so 1920 samples at 48 kHz, the same
 * 40 ms as 1600. Its modem is already at 48 kHz, so unlike 1600 there is no
 * resampling on the way in.
 *
 * Analog: 256 samples = 5.3 ms. SSB, AM and NBFM do not care much, but CW
 * break-in does, and 40 ms of round trip delay is very audible on a key.
 */
#define BLOCK_SIZE_MAX     3840
#define BLOCK_SIZE_FREEDV  3840
#define BLOCK_SIZE_2400B   3840
#define BLOCK_SIZE_700D    3840
#define BLOCK_SIZE_700E    3840
#define BLOCK_SIZE_ANALOG   512

/*
 * Everything that varies per mode, in one table (modes.c) rather than
 * scattered through if-chains - a front-panel menu only has to index this,
 * and adding a mode is one row rather than edits in five places.
 *
 * Ported from firmware/Core/Src/main.c (mode_cfg[] + the MODE_* defines) -
 * F746 had this table inline in main.c; split out here since main.c mixed
 * mode state, filter instances, and RX/TX signal chains in one file.
 */

#define MODE_NBFM 0
#define MODE_USB  1
#define MODE_LSB  2
#define MODE_FREEDV 3   // FreeDV 1600, received as USB
#define MODE_FREEDV_2400B 4  // FreeDV 2400B, through a normal FM audio path
#define MODE_FREEDV_700D  5  // FreeDV 700D, OFDM + LDPC on SSB for weak signals
#define MODE_FREEDV_700E  6  // FreeDV 700E, shorter frame than 700D, faster reacquire
#define MODE_AM           7  // AM, envelope detection off the complex baseband
#define MODE_CW            8 // CW, SSB demod into a narrow filter at the tone pitch

extern int MODE;   // current mode, default MODE_USB

uint32_t    block_size_for(int mode);
int         mode_is_freedv(int mode);
int         chain_mode_for(int mode);   /* FREEDV_CHAIN_MODE_*, or -1 if not FreeDV */
int         mode_is_buffered(int mode);
const char *mode_name(int mode);
int         mode_count(void);

/* radio_set_mode() (stops DMA, applies the new mode/block_size, reflushes
   filter state, restarts DMA) stays declared in main.h, not here - it's
   CM7-local dispatch glue that touches ADC/DAC handles and the FreeDV
   chain, not mode data. On F746, hmi.c called it directly (same core); on
   H7, hmi.c lives on CM4 and can't call a CM7 function directly across
   cores - it goes through the IPC mailbox instead (see ipc.h), which calls
   radio_set_mode() on CM7's side when a mode-change command arrives. */

#endif
