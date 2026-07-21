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

#endif
