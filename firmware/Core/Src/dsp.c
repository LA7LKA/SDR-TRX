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

void agc_block(float *x, int n, agc_t *st)
{
    agc_block_cfg(x, n, st, 0.25f, 0.02f);
}

void agc_block_cfg(float *x, int n, agc_t *st, float attack, float decay)
{
    const float target     = 0.5f;   /* leaves headroom before the DAC clips */
    const float gain_min   = 0.1f;
    const float gain_max   = 50.0f;

    float peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float a = fabsf(x[i]);
        if (a > peak) peak = a;
    }

    float g0 = st->gain;

    if (peak > 1e-6f)
    {
        float want = target / peak;

        if (want < gain_min) want = gain_min;
        if (want > gain_max) want = gain_max;

        st->gain += (want - st->gain) * ((want < st->gain) ? attack : decay);
    }

    /*
     * Ramp the gain across the block rather than switching it at the boundary.
     * A gain step is an amplitude discontinuity, and an amplitude
     * discontinuity is exactly what a click is -- at 5.3 ms blocks that would
     * be up to 188 of them a second.
     */
    float dg = (st->gain - g0) / (float)n;

    for (int i = 0; i < n; i++)
        x[i] *= g0 + dg * (float)i;
}

// -------------------- FM Discriminator (data) --------------------
void fm_discriminate(const float *I_in, const float *Q_in, float *out, int n)
{
    static float prev_I = 0.0f;
    static float prev_Q = 0.0f;

    static dc_block_t dc = {0};

    const float inv_pi = 1.0f / (float)M_PI;

    for (int i = 0; i < n; i++)
    {
        float I = I_in[i];
        float Q = Q_in[i];

        float re = I * prev_I + Q * prev_Q;
        float im = I * prev_Q - Q * prev_I;

        prev_I = I;
        prev_Q = Q;

        // atan2 spans +-pi; normalise so a full-scale deviation is +-1.0 and
        // the modem input cannot clip.
        out[i] = dc_block(atan2f(im, re), &dc) * inv_pi;
    }
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
