#include "dsp.h"
#include <math.h>

#define TWO_PI 6.28318530718f

// -------------------- NCO --------------------
void nco_init(nco_t *q, float freq_hz, float fs_hz)
{
    q->fs    = fs_hz;
    q->freq  = freq_hz;
    q->phase = 0.0f;
    q->dphi  = TWO_PI * freq_hz / fs_hz;
}

void nco_set_freq(nco_t *q, float freq_hz)
{
    q->freq = freq_hz;
    q->dphi = TWO_PI * freq_hz / q->fs;
}

void nco_step(nco_t *q)
{
    q->phase += q->dphi;
    if (q->phase >= TWO_PI)
        q->phase -= TWO_PI;
}

void nco_get_iq(nco_t *q, float *I, float *Q)
{
    *I = cosf(q->phase);
    *Q = sinf(q->phase);
}

// -------------------- DC Block --------------------
float dc_block(float x, dc_block_t *st)
{
    const float a = 0.995f;
    float y = x - st->x_prev + a * st->y_prev;

    st->x_prev = x;
    st->y_prev = y;

    return y;
}

// -------------------- Deemphasis --------------------
float fm_deemph(float x, deemph_t *st)
{
    const float fs  = 48000.0f;
    const float tau = 0.00050f;
    const float a   = expf(-1.0f / (fs * tau));

    float y = a * st->y_prev + (1.0f - a) * x;
    st->y_prev = y;

    return y;
}

// -------------------- AGC --------------------
float agc_process(float x, agc_t *st)
{
    float absx = fabsf(x);

    if (absx > 0.1f)
        st->gain *= 0.9995f;
    else
        st->gain *= 1.0005f;

    if (st->gain < 0.1f) st->gain = 0.1f;
    if (st->gain > 10.0f) st->gain = 10.0f;

    return x * st->gain;
}

// -------------------- FM Demod --------------------
void nbfm_demod(float *I_in, float *Q_in, float *audio_out, int n)
{
    static float prev_I = 0.0f;
    static float prev_Q = 0.0f;

    static dc_block_t dc_fm = {0};
    static deemph_t   deemph_fm = {0};
    static agc_t      agc_fm = {1.0f};

    for (int i = 0; i < n; i++)
    {
        float I = I_in[i];
        float Q = Q_in[i];

        float re = I * prev_I + Q * prev_Q;
        float im = I * prev_Q - Q * prev_I;

        float dphi = atan2f(im, re);

        prev_I = I;
        prev_Q = Q;

        float dc_removed = dc_block(dphi, &dc_fm);
        float deemph_out = fm_deemph(dc_removed, &deemph_fm);
        float agc_out    = agc_process(deemph_out, &agc_fm);

        audio_out[i] = agc_out; // Skalering for å unngå clipping
    }
}
