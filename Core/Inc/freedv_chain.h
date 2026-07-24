#ifndef FREEDV_CHAIN_H
#define FREEDV_CHAIN_H

#include <stdint.h>

/*
 * FreeDV 1600 receive chain.
 *
 * Sits behind the existing SSB demodulator: FreeDV is transmitted as an
 * ordinary SSB signal, so the audio coming out of ssb_process_block() is
 * already the modem passband signal. This module resamples that 48 kHz audio
 * down to the 8 kHz FreeDV expects, runs the demodulator, and hands back
 * decoded speech at 8 kHz.
 *
 * Rate handling is decoupled on both ends:
 *   - the 48 kHz side may push any block length (1024 is not a multiple of 6)
 *   - freedv_rx() consumes a varying number of samples per call (clock
 *     tracking), so input is buffered until freedv_nin() is satisfied
 *
 * 1600, 700D and 700E are HF modes carried on SSB and their modems run at
 * 8 kHz, so
 * the input is decimated 48k -> 8k. 2400B is designed to pass through a
 * commodity FM radio's audio path and its modem runs at 48 kHz natively, so
 * its input goes straight through. Both decode speech at
 * 8 kHz, so playback is interpolated back up either way.
 *
 * Returns 0 on success, -1 if codec2 failed to allocate.
 */
#define FREEDV_CHAIN_MODE_1600   0
#define FREEDV_CHAIN_MODE_2400B  1
#define FREEDV_CHAIN_MODE_700D   2
#define FREEDV_CHAIN_MODE_700E   3

int freedv_chain_init(int chain_mode);

/* Feed normalised (+-1.0) SSB audio at 48 kHz. */
void freedv_chain_put_audio48(const float *audio48, int n);

/*
 * Pull decoded speech at 8 kHz. Returns the number of samples written,
 * which is 0 until the modem has synced and produced a frame.
 */
int freedv_chain_get_speech8(int16_t *speech_out, int max);

/*
 * Pull decoded speech interpolated back up to 48 kHz, normalised to +-1.0 so
 * it drops straight into the existing audio gain and DAC stage.
 *
 * Always writes exactly n samples. Anything not covered by decoded speech
 * (no sync yet, or waiting on the next frame) is written as silence, so the
 * DAC never runs on stale data. The return value is the number of real
 * samples written, which is useful for spotting underruns.
 */
int freedv_chain_get_speech48(float *audio48, int n);

/*
 * Drop everything buffered and make the modem re-acquire. Call on a mode
 * change: the resamplers and FIFOs would otherwise carry audio from the old
 * mode, at the old block size, into the new stream.
 */
void freedv_chain_reset(void);

/* Non-zero once the modem has acquired sync. */
int freedv_chain_synced(void);

/* Signal quality in dB, valid while synced. */
float freedv_chain_snr(void);

#endif /* FREEDV_CHAIN_H */
