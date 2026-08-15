#include "rx_chain.h"
#include "modes.h"
#include "filters.h"
#include "freedv_chain.h"
#include "stm32h7xx.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

/* Shared scratch pool and RX telemetry - both owned by main.c (see the
   comment there for why: dsp_scratch is shared with tx_chain.c, the two
   never live at the same time since RX/TX are half duplex; the telemetry
   globals are what the CM4 HMI reads for the S-meter, via the IPC mailbox
   on H7 rather than a direct read like F746's single-core hmi.c did). */
#define SCRATCH_N (BLOCK_SIZE_MAX / 2)
extern float dsp_scratch[10 * SCRATCH_N];

extern volatile uint32_t rx_blocks;
extern volatile float    rx_peak;
extern volatile float    rx_rms;
extern volatile float    rx_env_min;
extern volatile float    rx_env_avg;
extern volatile float    rx_adc_peak;
extern volatile uint32_t rx_fdv_cycles_max;
extern volatile uint32_t rx_cycles_max;
extern int freedv_ok;

void process_block(const uint16_t *in, uint16_t *out, int n)
{
    uint32_t t0 = DWT->CYCCNT;

    if (MODE == MODE_NBFM)
        nbfm_process_block(in, out, n);
    else if (MODE == MODE_FREEDV_2400B)
        freedv2400b_process_block(in, out, n);   /* FM path, not SSB */
    else if (MODE == MODE_AM)
        am_process_block(in, out, n);            /* own path, no Hilbert */
    else
        ssb_process_block(in, out, n);           /* USB/LSB/CW/FreeDV-on-SSB */

    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > rx_cycles_max)
        rx_cycles_max = dt;
}

void ssb_process_block(const uint16_t *in, uint16_t *out, int n)
{
    // Carved out of the shared scratch pool; see dsp_scratch above.
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];

    float *IQ_in     = &dsp_scratch[3 * SCRATCH_N];   // interleaved I/Q,   2n
    float *NCO_buf   = &dsp_scratch[5 * SCRATCH_N];   // interleaved cos/sin, 2n
    float *IQ_mix    = &dsp_scratch[7 * SCRATCH_N];   // interleaved I/Q,   2n
    float *ssb_raw   = &dsp_scratch[9 * SCRATCH_N];   // FreeDV fallback audio, see MODE_FREEDV below

    // ---------------------------------------------------------
    // 1) ADC -> normalised I
    // ---------------------------------------------------------
    float adc_peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        I_buf[i] = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(I_buf[i]);
        if (a > adc_peak) adc_peak = a;
    }

    rx_adc_peak = adc_peak;

    // ---------------------------------------------------------
    // 2) Hilbert -> Q (for SSB and FM)
    // ---------------------------------------------------------
    arm_fir_f32(&hilbert, I_buf, Q_buf, n);

    // ---------------------------------------------------------
    // 3) Interleaved I/Q
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] = I_buf[i];
        IQ_in[2*i + 1] = Q_buf[i];
    }

    // ---------------------------------------------------------
    // 4) NCO -> interleaved cos/sin
    // ---------------------------------------------------------
    nco_block_iq(NCO_buf, n);

    // ---------------------------------------------------------
    // 5) Complex mixing
    // ---------------------------------------------------------
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    // ---------------------------------------------------------
    // 6) Split I/Q after mixing
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        I_buf[i] = IQ_mix[2*i + 0];
        Q_buf[i] = IQ_mix[2*i + 1];
    }

    // ---------------------------------------------------------
    // 7) Pre-demod IF filter
    // ---------------------------------------------------------
    arm_fir_f32(&pre_fm_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm_Q, Q_buf, Q_buf, n);

    // ---------------------------------------------------------
    // 8) Mode-based demod
    // ---------------------------------------------------------
    if (MODE == MODE_USB)
    {
        // USB = I - Q  (swapped to match how the sideband lands on the air
        // after the up-conversion, confirmed against an FT-857D)
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] - Q_buf[i];

        /* Post-demod level, same pattern as the CW branch below - this is
           what the HMI's S-meter reads (via the IPC mailbox on H7) - it
           needs to reflect the tuned signal, not just the raw pre-mix ADC
           swing. */
        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
            sumsq += audio_buf[i] * audio_buf[i];
        }
        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;
    }
    else if (MODE == MODE_LSB)
    {
        // LSB = I + Q
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] + Q_buf[i];

        /* Same reasoning as MODE_USB above. */
        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
            sumsq += audio_buf[i] * audio_buf[i];
        }
        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;
    }
    else if (MODE == MODE_CW)
    {
        /*
         * CW is just SSB into a narrow filter. Tuning is what places the
         * carrier at the wanted pitch; the filter is centred there rather
         * than at zero, which is why the pitch is a system setting and not
         * simply a bandwidth.
         */
        static agc_t agc_cw = {1.0f};

        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] + Q_buf[i];

        arm_biquad_cascade_df1_f32(&cw_bpf, audio_buf, audio_buf, n);

        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
            sumsq += audio_buf[i] * audio_buf[i];
        }
        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;

        /* Slow decay so the gain does not wind up between elements. */
        agc_block_cfg(audio_buf, n, &agc_cw, 0.25f, 0.002f);
    }
    else if (MODE == MODE_FREEDV || MODE == MODE_FREEDV_700D
                                 || MODE == MODE_FREEDV_700E)
    {
        // FreeDV rides on an ordinary SSB signal, so demodulate as USB first.
        float peak = 0.0f, sumsq = 0.0f;

        for (int i = 0; i < n; i++)
        {
            audio_buf[i] = I_buf[i] + Q_buf[i];

            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
            sumsq += audio_buf[i] * audio_buf[i];
        }

        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;

        // Keep the plain-SSB audio so we can fall back to it below - this is
        // what makes "listen on SSB, switch over once FreeDV syncs" work:
        // audio_buf gets overwritten with decoded speech (or noise, while
        // still hunting for sync), so save the version before that happens.
        memcpy(ssb_raw, audio_buf, n * sizeof(float));

        // 48k -> 8k -> FreeDV demod -> 8k -> 48k
        if (freedv_ok)
        {
            uint32_t t0 = DWT->CYCCNT;

            freedv_chain_put_audio48(audio_buf, n);
            freedv_chain_get_speech48(audio_buf, n);

            uint32_t dt = DWT->CYCCNT - t0;
            if (dt > rx_fdv_cycles_max)
                rx_fdv_cycles_max = dt;
        }

        // Not synced yet (or freedv_open() itself failed): play the SSB
        // audio instead of silence/decoder noise, same idea as an analog
        // squelch - you hear the signal you're tuned to either way, just
        // as decoded speech once sync locks.
        if (!freedv_ok || !freedv_chain_synced())
            memcpy(audio_buf, ssb_raw, n * sizeof(float));
    }
    else
    {
        for (int i = 0; i < n; i++)
            audio_buf[i] = 0.0f;
    }

    // ---------------------------------------------------------
    // 9) Audio gain before DAC
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
        audio_buf[i] *= 1500.0f;

    // ---------------------------------------------------------
    // 10) DAC output
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

void nbfm_process_block(const uint16_t *in, uint16_t *out, int n)
{
    // Carved out of the shared scratch pool; see dsp_scratch above.
    float *I_if          = &dsp_scratch[0 * SCRATCH_N];
    float *Q_if          = &dsp_scratch[1 * SCRATCH_N];
    float *audio_fm      = &dsp_scratch[2 * SCRATCH_N];   // FM-demod audio
    float *audio_lpf_out = &dsp_scratch[3 * SCRATCH_N];

    float *IQ_in         = &dsp_scratch[4 * SCRATCH_N];   // interleaved I/Q,   2n
    float *NCO_buf       = &dsp_scratch[6 * SCRATCH_N];   // interleaved cos/sin, 2n
    float *IQ_mix        = &dsp_scratch[8 * SCRATCH_N];   // interleaved I/Q,   2n

    // -----------------------------
    // 1) ADC -> normalised IF (I)
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        I_if[i] = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;
    }

    arm_fir_f32(&hilbert, I_if, Q_if, n);

    // -----------------------------
    // 2) Interleaved I/Q
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] = I_if[i];
        IQ_in[2*i + 1] = Q_if[i];
    }

    // -----------------------------
    // 3) NCO -> interleaved cos/sin
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        float cs = cosf(nco_if.phase);
        float sn = sinf(nco_if.phase);
        nco_step(&nco_if);

        NCO_buf[2*i + 0] = cs;
        NCO_buf[2*i + 1] = sn;
    }

    // -----------------------------
    // 4) Complex mixing (IF -> baseband FM)
    // -----------------------------
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    // -----------------------------
    // 5) Split I/Q after mixing
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        I_if[i] = IQ_mix[2*i + 0];
        Q_if[i] = IQ_mix[2*i + 1];
    }

    // -----------------------------
    // 6) Pre-demod IF filter (complex)
    // -----------------------------
    arm_fir_f32(&pre_fm_I, I_if, I_if, n);
    arm_fir_f32(&pre_fm_Q, Q_if, Q_if, n);

    // -----------------------------
    // 7) FM demodulator
    // -----------------------------
    nbfm_demod(I_if, Q_if, audio_fm, n);

    // -----------------------------
    // 8) Audio LPF after FM demod
    // -----------------------------
    arm_fir_f32(&audio_lpf, audio_fm, audio_lpf_out, n);

    // Post-demod level for the HMI's S-meter (via IPC on H7)
    {
        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float a = fabsf(audio_lpf_out[i]);
            if (a > peak) peak = a;
            sumsq += audio_lpf_out[i] * audio_lpf_out[i];
        }
        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;
    }

    // -----------------------------
    // 9) Gain + DAC scaling
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        float y = audio_lpf_out[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

/*
 * FreeDV 2400A/2400B receive.
 *
 * 2400A rides inside an ordinary FM channel, so this is the FM path, not
 * the SSB one: mix the 12 kHz IF down, filter the channel wide enough to
 * keep the 4FSK tones intact, and FM demodulate. What comes out is the
 * modem's own 48 kHz signal, which goes straight to the demodulator
 * without resampling.
 *
 * Deliberately no post-demod audio LPF: the voice filter would cut the
 * 4800 Hz tone and the modem would never sync.
 */
void freedv2400b_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];

    float *IQ_in     = &dsp_scratch[3 * SCRATCH_N];
    float *NCO_buf   = &dsp_scratch[5 * SCRATCH_N];
    float *IQ_mix    = &dsp_scratch[7 * SCRATCH_N];
    float *nbfm_raw  = &dsp_scratch[9 * SCRATCH_N];   // FreeDV fallback audio, see below

    float adc_peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        I_buf[i] = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(I_buf[i]);
        if (a > adc_peak) adc_peak = a;
    }
    rx_adc_peak = adc_peak;

    /* Proper analytic pair: Q is a real Hilbert transform of I, and I is
       delayed by the same group delay so the two legs line up. Building it
       as (I_raw, filtered_I) - what the shared path above still does - combs
       the effective response into nulls; see filters.c. audio_buf is free
       scratch here, fm_discriminate() below is the first thing to write it. */
    arm_fir_f32(&hilbert_fm2400, I_buf, Q_buf, n);
    arm_fir_f32(&delay_fm2400,   I_buf, audio_buf, n);

    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] =  audio_buf[i];  /* I, delay-matched to Q */
        IQ_in[2*i + 1] = -Q_buf[i];      /* see sign note below */
    }

    nco_block_iq(NCO_buf, n);
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    for (int i = 0; i < n; i++)
    {
        I_buf[i] = IQ_mix[2*i + 0];
        Q_buf[i] = IQ_mix[2*i + 1];
    }

    // 2400B's own wider pair rather than the shared pre_fm_I/Q the voice
    // paths use: fmfsk runs Manchester at Rs=4800, so the first-order FM
    // sidebands land at +-4800 Hz and the voice filter is ~4 dB down
    // there (and the shared pre-mix bandpass ~25 dB down). Those sidebands
    // carry the data - see the block comment in filters.c.
    arm_fir_f32(&pre_fm2400_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm2400_Q, Q_buf, Q_buf, n);

    // Data discriminator, not the voice one: no de-emphasis, no voice AGC.
    fm_discriminate(I_buf, Q_buf, audio_buf, n);

    /*
     * Peak alone cannot tell an over-deviated signal from an occasional
     * noise spike, so track rms as well. A clean FM data signal sits
     * around peak/rms of 3-4; a much larger ratio means the discriminator
     * is spiking, and a peak near 1.0 with high rms means the deviation
     * itself is too big and atan2 is wrapping.
     */
    float peak = 0.0f;
    float sumsq = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio_buf[i]);
        if (a > peak) peak = a;
        sumsq += audio_buf[i] * audio_buf[i];
    }

    rx_peak = peak;
    rx_rms  = sqrtf(sumsq / (float)n);
    rx_blocks++;

    // Listen on plain NBFM until 2400B syncs, same idea (and same reason)
    // as the SSB-mode fallback in ssb_process_block(): audio_buf is about
    // to be overwritten with decoded speech or decoder noise, so save the
    // discriminator output first and fall back to it if not synced.
    memcpy(nbfm_raw, audio_buf, n * sizeof(float));

    if (freedv_ok)
    {
        uint32_t t0 = DWT->CYCCNT;

        freedv_chain_put_audio48(audio_buf, n);
        freedv_chain_get_speech48(audio_buf, n);

        uint32_t dt = DWT->CYCCNT - t0;
        if (dt > rx_fdv_cycles_max)
            rx_fdv_cycles_max = dt;
    }

    if (!freedv_ok || !freedv_chain_synced())
        memcpy(audio_buf, nbfm_raw, n * sizeof(float));

    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

/*
 * AM receive.
 *
 * Deliberately not routed through the SSB path. That one builds its
 * complex signal with a Hilbert transformer, which AM does not need:
 * mixing the real IF against cos and -sin gives I and Q directly, and the
 * existing channel filter removes the sum-frequency image. Skipping the
 * Hilbert removes both its quadrature error and its group delay, neither
 * of which the envelope detector can tolerate -- any I/Q imbalance turns
 * straight into amplitude error, and the whole point of AM is that the
 * amplitude is the signal.
 *
 * The envelope is then |I + jQ|, which needs no knowledge of the carrier
 * phase and, unlike SSB, is completely unaffected by a tuning offset.
 */
void am_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];
    float *NCO_buf   = &dsp_scratch[3 * SCRATCH_N];   /* interleaved, 2n */

    static dc_block_t dc_am  = {0};
    static agc_t      agc_am = {1.0f};

    float adc_peak = 0.0f;

    nco_block_iq(NCO_buf, n);

    for (int i = 0; i < n; i++)
    {
        float x = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(x);
        if (a > adc_peak) adc_peak = a;

        /* Quadrature downconversion: multiply by exp(-j*w*t). */
        I_buf[i] =  x * NCO_buf[2*i + 0];
        Q_buf[i] = -x * NCO_buf[2*i + 1];
    }

    rx_adc_peak = adc_peak;

    /* Removes the image at twice the IF that the mixing leaves behind. */
    arm_fir_f32(&pre_fm_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm_Q, Q_buf, Q_buf, n);

    float peak = 0.0f, sumsq = 0.0f;
    float env_min = 1e9f, env_sum = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float env = sqrtf(I_buf[i] * I_buf[i] + Q_buf[i] * Q_buf[i]);

        /*
         * env_min against env_avg says whether the transmitter sent a
         * carrier. Proper AM never lets the envelope reach zero, so
         * env_min stays near (1 - m) of the mean. If env_min collapses to
         * zero the envelope is being rectified, which is what a
         * suppressed carrier looks like and why the tone comes out at
         * twice its frequency.
         */
        if (env < env_min) env_min = env;
        env_sum += env;

        /* The carrier is a DC pedestal under the envelope; drop it. */
        audio_buf[i] = dc_block(env, &dc_am);

        float a = fabsf(audio_buf[i]);
        if (a > peak) peak = a;
        sumsq += audio_buf[i] * audio_buf[i];
    }

    rx_env_min = env_min;
    rx_env_avg = env_sum / (float)n;
    rx_peak = peak;
    rx_rms  = sqrtf(sumsq / (float)n);
    rx_blocks++;

    agc_block(audio_buf, n, &agc_am);

    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}
