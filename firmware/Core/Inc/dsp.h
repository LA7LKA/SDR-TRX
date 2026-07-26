#ifndef DSP_H
#define DSP_H
#include "stm32f7xx.h"      
#include "arm_math.h"

// -------------------- NCO --------------------
typedef struct {
    float phase;
    float dphi;
    float fs;
    float freq;
} nco_t;

void nco_init(nco_t *q, float freq_hz, float fs_hz);
void nco_set_freq(nco_t *q, float freq_hz);
void nco_step(nco_t *q);
void nco_get_iq(nco_t *q, float *I, float *Q);

// -------------------- DC Block --------------------
typedef struct {
    float y_prev;
    float x_prev;
} dc_block_t;

float dc_block(float x, dc_block_t *st);

// -------------------- Deemphasis --------------------
typedef struct {
    float y_prev;
} deemph_t;

float fm_deemph(float x, deemph_t *st);

// -------------------- AGC --------------------
typedef struct {
    float gain;
} agc_t;

float agc_process(float x, agc_t *st);

/*
 * Block AGC, for anything with a clean waveform.
 *
 * agc_process() moves its gain on every sample against a fixed threshold, so
 * on a tone the gain hunts several times per cycle and the varying gain across
 * the waveform is itself distortion. This derives one gain from the block's
 * peak and applies it uniformly, so the waveform is only scaled, never bent.
 * Attack is quick to catch a rising signal before it clips; decay is slow so
 * the background does not pump.
 */
void agc_block(float *x, int n, agc_t *st);

/*
 * As agc_block(), but with the time constants exposed. CW wants a much slower
 * decay than voice: with a fast one the gain winds up in the gaps between
 * elements and the keying itself starts to pump.
 */
void agc_block_cfg(float *x, int n, agc_t *st, float attack, float decay);

// -------------------- FM Demod --------------------
void nbfm_demod(float *I_in, float *Q_in, float *audio_out, int n);

/*
 * FM discriminator for data, as opposed to nbfm_demod() which is for voice.
 *
 * Voice FM wants de-emphasis and AGC; a modem wants neither. The 500 us
 * de-emphasis in nbfm_demod() rolls off 11.8 dB at 1200 Hz and 23.6 dB at
 * 4800 Hz, which tilts a data waveform badly across its own band, and the
 * voice AGC then drives the result into clipping.
 *
 * This is the raw phase difference with DC removed, scaled by 1/pi so the
 * output keeps the +-1.0 convention the rest of the chain uses.
 */
void fm_discriminate(const float *I_in, const float *Q_in, float *out, int n);

#endif
