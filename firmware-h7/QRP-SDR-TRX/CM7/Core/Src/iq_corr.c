/*
 * iq_corr.c - automatic (blind) RX I/Q correction. See iq_corr.h for the
 * idea; this file is the arithmetic.
 *
 * Per block:
 *   1. remove DC (running estimate, ~0.5 s)
 *   2. accumulate E[I^2], E[Q^2], E[I*Q] into running averages (~4 s)
 *   3. turn those into a gain/phase estimate and a 2x2 correction
 *   4. apply it, and measure the same statistics on the output so the
 *      result can be read back as an image-rejection figure
 *
 * Correction, with g and phi from iq_corr.h:
 *     I' = s * I
 *     Q' = s * (Q / g - I * sin(phi)) / cos(phi)
 * s = min(1, g) scales by the weaker channel, so a Q that is 8 dB low
 * pulls I down rather than Q up - nothing new can clip.
 *
 * Image rejection readout: a residual image coefficient k (z = s + k s*)
 * makes E[z^2] = 2k E[|z|^2], so |k| = |E[z^2]| / (2 E[|z|^2]) with
 * E[z^2] = E[I^2] - E[Q^2] + 2j E[I*Q]. It's an estimate with a floor of
 * roughly 50-60 dB from the finite averaging, not a lab measurement.
 *
 * TX (not done here): the DAC samples are ideal, so there's nothing to
 * estimate from. It needs the TX output fed back into the RX mixer (a
 * loopback at low level), the image measured on RX, and the inverse
 * pre-distortion applied before the DACs - same 2x2 form.
 */
#include "iq_corr.h"
#include <math.h>
#include <string.h>

volatile int32_t iqc_enable = 1;
volatile int32_t iqc_freeze = 0;
volatile int32_t iqc_reset  = 0;

volatile float iqc_gain_db    = 0.0f;
volatile float iqc_phase_deg  = 0.0f;
volatile float iqc_dc_i       = 0.0f;
volatile float iqc_dc_q       = 0.0f;
volatile float iqc_irr_in_db  = 0.0f;
volatile float iqc_irr_out_db = 0.0f;

#define TAU_DC_S     0.5f    /* DC tracking time constant */
#define TAU_STATS_S  4.0f    /* gain/phase averaging time constant */
#define POWER_FLOOR  1e-9f   /* ~-90 dBFS: below this, don't adapt */
#define G_MIN        0.25f   /* +-12 dB: beyond this it's a fault, */
#define G_MAX        4.0f    /*   not something to correct away   */
#define SIN_MAX      0.5f    /* +-30 deg, same reasoning */
#define IRR_MAX_DB   99.0f

typedef struct {
    float ii, qq, iq;        /* running E[I^2], E[Q^2], E[I*Q] */
} stats_t;

static struct {
    float    dc_i, dc_q;
    stats_t  in, out;
    uint32_t blocks;         /* since reset, for a fast start */
    float    ci, cqq, cqi;   /* I' = ci*I;  Q' = cqq*Q + cqi*I */
} st;

void iq_corr_init(void)
{
    memset(&st, 0, sizeof st);
    st.ci  = 1.0f;
    st.cqq = 1.0f;
    st.cqi = 0.0f;
    iqc_gain_db = iqc_phase_deg = 0.0f;
    iqc_dc_i = iqc_dc_q = 0.0f;
    iqc_irr_in_db = iqc_irr_out_db = 0.0f;
}

/* Running-average weight for a time constant, with a cumulative average
   for the first tau so it converges in tau after a reset, not 3*tau. */
static float avg_weight(uint32_t n, float fs, float tau)
{
    float a = (float)n / (fs * tau);
    float warm = 1.0f / (float)st.blocks;
    if (warm > a) a = warm;
    return (a > 1.0f) ? 1.0f : a;
}

static void stats_update(stats_t *s, float ii, float qq, float iq, float a)
{
    s->ii += a * (ii - s->ii);
    s->qq += a * (qq - s->qq);
    s->iq += a * (iq - s->iq);
}

static float irr_db(const stats_t *s)
{
    float p = s->ii + s->qq;
    if (p < POWER_FLOOR)
        return 0.0f;
    float re = s->ii - s->qq;
    float im = 2.0f * s->iq;
    float k = sqrtf(re * re + im * im) / (2.0f * p);
    if (k < 1e-5f)
        return IRR_MAX_DB;
    float db = -20.0f * log10f(k);
    return (db > IRR_MAX_DB) ? IRR_MAX_DB : db;
}

static void estimate(void)
{
    const stats_t *s = &st.in;
    if (s->ii < POWER_FLOOR || s->qq < POWER_FLOOR)
        return;                              /* no signal: keep what we have */

    float g = sqrtf(s->qq / s->ii);
    float sinp = s->iq / sqrtf(s->ii * s->qq);
    if (g < G_MIN) g = G_MIN;
    if (g > G_MAX) g = G_MAX;
    if (sinp >  SIN_MAX) sinp =  SIN_MAX;
    if (sinp < -SIN_MAX) sinp = -SIN_MAX;
    float cosp = sqrtf(1.0f - sinp * sinp);

    float sc = (g < 1.0f) ? g : 1.0f;
    st.ci  = sc;
    st.cqq = sc / (g * cosp);
    st.cqi = -sc * sinp / cosp;

    iqc_gain_db   = 20.0f * log10f(g);
    iqc_phase_deg = asinf(sinp) * (180.0f / 3.14159265f);
}

/* Clamp in float, then round half away from zero - compiles to VCVT,
   where lrintf() was a library call per sample. */
static inline int16_t sat16(float x)
{
    if (x >  32767.0f) x =  32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)(int32_t)((x >= 0.0f) ? x + 0.5f : x - 0.5f);
}

void iq_corr_process(int16_t *iq, uint32_t n, float fs)
{
    if (n == 0)
        return;
    if (iqc_reset || st.blocks == 0)     /* first call: unity coefficients */
    {
        iq_corr_init();
        iqc_reset = 0;
    }
    st.blocks++;

    const float k = 1.0f / 32768.0f;

    /* Pass 1: DC and input statistics, DC removed with the current
       estimate (updated afterwards - a block of lag is irrelevant). */
    float sum_i = 0, sum_q = 0, sii = 0, sqq = 0, siq = 0;
    for (uint32_t m = 0; m < n; m++)
    {
        float i = (float)iq[2 * m] * k;
        float q = (float)iq[2 * m + 1] * k;
        sum_i += i;
        sum_q += q;
        i -= st.dc_i;
        q -= st.dc_q;
        sii += i * i;
        sqq += q * q;
        siq += i * q;
    }
    float inv_n = 1.0f / (float)n;
    float a_dc = avg_weight(n, fs, TAU_DC_S);
    st.dc_i += a_dc * (sum_i * inv_n - st.dc_i);
    st.dc_q += a_dc * (sum_q * inv_n - st.dc_q);

    float a = avg_weight(n, fs, TAU_STATS_S);
    stats_update(&st.in, sii * inv_n, sqq * inv_n, siq * inv_n, a);
    if (!iqc_freeze)
        estimate();

    /* Pass 2: apply (or not), measuring the output either way. */
    sii = sqq = siq = 0;
    if (iqc_enable)
    {
        const float dci = st.dc_i, dcq = st.dc_q;
        const float ci = st.ci, cqq = st.cqq, cqi = st.cqi;
        for (uint32_t m = 0; m < n; m++)
        {
            float i = (float)iq[2 * m] * k - dci;
            float q = (float)iq[2 * m + 1] * k - dcq;
            float io = ci * i;
            float qo = cqq * q + cqi * i;
            sii += io * io;
            sqq += qo * qo;
            siq += io * qo;
            iq[2 * m]     = sat16(io * 32768.0f);
            iq[2 * m + 1] = sat16(qo * 32768.0f);
        }
        stats_update(&st.out, sii * inv_n, sqq * inv_n, siq * inv_n, a);
    }
    else
    {
        st.out = st.in;      /* output is the raw input */
    }

    iqc_dc_i       = st.dc_i;
    iqc_dc_q       = st.dc_q;
    iqc_irr_in_db  = irr_db(&st.in);
    iqc_irr_out_db = irr_db(&st.out);
}
