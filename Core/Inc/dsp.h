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
