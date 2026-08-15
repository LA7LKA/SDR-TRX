#include "filters.h"
#include "modes.h"
#include <math.h>

#define TWO_PI 6.28318530718f

// -------------------- Shared downconversion NCOs --------------------
nco_t nco;
nco_t nco_if;

/*
 * Fill an interleaved cos/sin buffer for the IF mixer.
 *
 * Recursive phasor rotation rather than cosf()/sinf() per sample: two libm
 * calls per sample was one of the larger costs in the block. Magnitude drifts
 * slowly, so it is renormalised once per call instead of per sample.
 */
void nco_block_iq(float *NCO_buf, int n)
{
    static float osc_re = 1.0f, osc_im = 0.0f;
    static float step_re = 1.0f, step_im = 0.0f;
    static float step_dphi = 0.0f;

    if (step_dphi != nco.dphi)
    {
        step_dphi = nco.dphi;
        step_re   = cosf(nco.dphi);
        step_im   = sinf(nco.dphi);
    }

    for (int i = 0; i < n; i++)
    {
        NCO_buf[2*i + 0] = osc_re;
        NCO_buf[2*i + 1] = osc_im;

        float nre = osc_re * step_re - osc_im * step_im;
        float nim = osc_re * step_im + osc_im * step_re;

        osc_re = nre;
        osc_im = nim;
    }

    float mag2 = osc_re * osc_re + osc_im * osc_im;
    float g    = 1.5f - 0.5f * mag2;   // one Newton step toward 1/sqrt(mag2)

    osc_re *= g;
    osc_im *= g;
}

// -------------------- RX Hilbert (SSB/NBFM/FreeDV-2400B) --------------------
/*
FIR filter designed with http://t-filter.appspot.com
sampling frequency: 48000 Hz
* 0 Hz - 6000 Hz: gain = 0, desired attenuation = -60 dB, actual attenuation = -50.41488654693752 dB
* 8000 Hz - 16000 Hz: gain = 1, desired ripple = 5 dB, actual ripple = 15.449541453020096 dB
* 18000 Hz - 24000 Hz: gain = 0, desired attenuation = -60 dB, actual attenuation = -50.41488654693752 dB
*/
#define HILBERT_TAPS 17

static float hilbert_coeffs[HILBERT_TAPS] = {
  0.02352204878641468,
  -7.964549159078516e-16,
  -0.094806149454076,
  5.916427478655016e-16,
  0.21352320600109478,
  -7.4996177568773785e-16,
  -0.3325693585500034,
  4.3317065620714974e-17,
  0.3830170947566392,
  4.3317065620714974e-17,
  -0.3325693585500034,
  -7.4996177568773785e-16,
  0.21352320600109478,
  5.916427478655016e-16,
  -0.094806149454076,
  -7.964549159078516e-16,
  0.02352204878641468
};

static float hilbert_state[HILBERT_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 hilbert;

// -------------------- IF pre-filter (I/Q, shared) --------------------
#define PRE_DEMOD_TAPS 73

static float pre_demod_coeffs[PRE_DEMOD_TAPS] = {
  -0.0004930375303321366,
  0.0001064813438661428,
  0.0005748457570259578,
  0.0011898445128213748,
  0.0016502280581233504,
  0.001605207031383133,
  0.000831446560674343,
  -0.0005973708354363059,
  -0.002228627696676951,
  -0.0033377455060532744,
  -0.0032076016149152073,
  -0.0014978471407907723,
  0.001459284866735388,
  0.004620535335059582,
  0.00653528877009168,
  0.0059370991846072535,
  0.00240691382833557,
  -0.003197318174051976,
  -0.008810233649088216,
  -0.0118211865758766,
  -0.010154929160148466,
  -0.003367069515417546,
  0.006744477803887592,
  0.016400388788851774,
  0.021048898446146783,
  0.017199001020187076,
  0.0042202354750242735,
  -0.014651068016007818,
  -0.032657469215897225,
  -0.04131750763191529,
  -0.03320662673344374,
  -0.004808839248635039,
  0.04161063532888555,
  0.09790380688954856,
  0.1519161665559878,
  0.1908448312364271,
  0.20501884795915196,
  0.1908448312364271,
  0.1519161665559878,
  0.09790380688954856,
  0.04161063532888555,
  -0.004808839248635039,
  -0.03320662673344374,
  -0.04131750763191529,
  -0.032657469215897225,
  -0.014651068016007818,
  0.0042202354750242735,
  0.017199001020187076,
  0.021048898446146783,
  0.016400388788851774,
  0.006744477803887592,
  -0.003367069515417546,
  -0.010154929160148466,
  -0.0118211865758766,
  -0.008810233649088216,
  -0.003197318174051976,
  0.00240691382833557,
  0.0059370991846072535,
  0.00653528877009168,
  0.004620535335059582,
  0.001459284866735388,
  -0.0014978471407907723,
  -0.0032076016149152073,
  -0.0033377455060532744,
  -0.002228627696676951,
  -0.0005973708354363059,
  0.000831446560674343,
  0.001605207031383133,
  0.0016502280581233504,
  0.0011898445128213748,
  0.0005748457570259578,
  0.0001064813438661428,
  -0.0004930375303321366
};

static float pre_fm_state_I[PRE_DEMOD_TAPS + BLOCK_SIZE_MAX/2];
static float pre_fm_state_Q[PRE_DEMOD_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 pre_fm_I;
arm_fir_instance_f32 pre_fm_Q;

/* -------------------- FreeDV 2400B front-end (FM data) --------------------
 *
 * 2400B gets its own pre-mix bandpass and post-mix lowpass rather than
 * sharing 'hilbert'/'pre_fm_I/Q' with NBFM voice and the SSB modes, because
 * its FM signal is far wider than any of those need.
 *
 * fmfsk_create(48000, 2400) sets Rs = Rb*2 = 4800 (Manchester), so the
 * modulating waveform carries content to 4800 Hz - the same 4800 Hz tone
 * freedv2400b_process_block() already refuses to put an audio LPF in front
 * of. At the 2.5 kHz deviation this bench runs, the modulation index is
 * 2500/4800 ~= 0.52, so the information-bearing first-order FM sidebands
 * sit at 12 kHz +-4800 Hz, i.e. 7.2 kHz and 16.8 kHz.
 *
 * Two separate defects made that unusable, both measured on hardware:
 *
 *  1. The shared 'hilbert' bandpass (8-16 kHz) is ~25 dB down at exactly
 *     those two frequencies, so the sidebands carrying the data were being
 *     thrown away. Widening it alone got SNR 1.9 -> 5.6 dB but no further.
 *
 *  2. The deeper one. Despite its name 'hilbert' is a *symmetric*
 *     linear-phase bandpass (0 deg phase, centre tap 0.383), not a Hilbert
 *     transformer, and the chains build their pseudo-analytic pair as
 *     Z = I_raw + j*(h * I_raw). Because h delays by D = (taps-1)/2 while
 *     I_raw is not delayed at all, the two legs are misaligned by D
 *     samples, and |1 + j*H(f)| combs: nulls wherever the relative phase
 *     hits 270 deg. Measured -16.9 dB at 10.4 kHz for the 17-tap filter,
 *     and a 45-tap widening only moved the null to 12.55 kHz at -43.7 dB
 *     (more taps = more delay = more in-band nulls). This is also exactly
 *     why the SSB modes never noticed: they occupy ~12-14 kHz and sit
 *     between nulls, while 2400B spans 7.2-16.8 kHz and lands right on
 *     them.
 *
 * Fix is the textbook construction: a true Hilbert transformer (Type III
 * FIR - antisymmetric, so 90 deg by construction) for Q, and a matched
 * pure-delay FIR of the same length for I so both legs line up. Effective
 * |I_delayed + jQ| is then flat to 0.00 dB ripple across 7.2-16.8 kHz
 * instead of 15-44 dB of comb nulls. The post-mix lowpass (-0.19 dB at
 * +-4800 Hz) does channel selection as before.
 */
#define HILB_FM2400_TAPS 31
#define PRE_FM2400_TAPS  73

static float hilbert_fm2400_coeffs[31] = {
    +5.8899160768e-19f, -0.0000000000e+00f, -8.2063813022e-04f, -0.0000000000e+00f,
    -4.4605296657e-03f, -0.0000000000e+00f, -1.4201582545e-02f, -0.0000000000e+00f,
    -3.5833728487e-02f, -0.0000000000e+00f, -8.0214091318e-02f, -0.0000000000e+00f,
    -1.8021217269e-01f, -0.0000000000e+00f, -6.2526084396e-01f, +0.0000000000e+00f,
    +6.2526084396e-01f, +0.0000000000e+00f, +1.8021217269e-01f, +0.0000000000e+00f,
    +8.0214091318e-02f, +0.0000000000e+00f, +3.5833728487e-02f, +0.0000000000e+00f,
    +1.4201582545e-02f, +0.0000000000e+00f, +4.4605296657e-03f, +0.0000000000e+00f,
    +8.2063813022e-04f, +0.0000000000e+00f, -5.8899160768e-19f,
};

static float delay_fm2400_coeffs[31] = {
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +1.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
    +0.0000000000e+00f, +0.0000000000e+00f, +0.0000000000e+00f,
};

static float pre_fm2400_coeffs[73] = {
    -1.3524500383e-34f, +4.4197247898e-06f, +2.5947247167e-05f, +4.3100046075e-05f,
    -1.1146086630e-19f, -1.3264502054e-04f, -2.8634412630e-04f, -2.9326709122e-04f,
    +4.9574642285e-19f, +5.5391635743e-04f, +1.0371312394e-03f, +9.5298635442e-04f,
    -1.2669058456e-18f, -1.5390306596e-03f, -2.7158542814e-03f, -2.3719952865e-03f,
    +2.5143257193e-18f, +3.5258934882e-03f, +6.0125200724e-03f, +5.0946417991e-03f,
    -4.2066018067e-18f, -7.2044435551e-03f, -1.2044084792e-02f, -1.0040018024e-02f,
    +6.1396206362e-18f, +1.3902246810e-02f, +2.3157106310e-02f, +1.9345631638e-02f,
    -7.9611785994e-18f, -2.7554104767e-02f, -4.7375795250e-02f, -4.1620583534e-02f,
    +9.2691691302e-18f, +7.2943998974e-02f, +1.5717812509e-01f, +2.2437727556e-01f,
    +2.5000000000e-01f, +2.2437727556e-01f, +1.5717812509e-01f, +7.2943998974e-02f,
    +9.2691691302e-18f, -4.1620583534e-02f, -4.7375795250e-02f, -2.7554104767e-02f,
    -7.9611785994e-18f, +1.9345631638e-02f, +2.3157106310e-02f, +1.3902246810e-02f,
    +6.1396206362e-18f, -1.0040018024e-02f, -1.2044084792e-02f, -7.2044435551e-03f,
    -4.2066018067e-18f, +5.0946417991e-03f, +6.0125200724e-03f, +3.5258934882e-03f,
    +2.5143257193e-18f, -2.3719952865e-03f, -2.7158542814e-03f, -1.5390306596e-03f,
    -1.2669058456e-18f, +9.5298635442e-04f, +1.0371312394e-03f, +5.5391635743e-04f,
    +4.9574642285e-19f, -2.9326709122e-04f, -2.8634412630e-04f, -1.3264502054e-04f,
    -1.1146086630e-19f, +4.3100046075e-05f, +2.5947247167e-05f, +4.4197247898e-06f,
    -1.3524500383e-34f,
};
static float hilbert_fm2400_state[HILB_FM2400_TAPS + BLOCK_SIZE_MAX/2];
static float delay_fm2400_state[HILB_FM2400_TAPS + BLOCK_SIZE_MAX/2];
static float pre_fm2400_state_I[PRE_FM2400_TAPS + BLOCK_SIZE_MAX/2];
static float pre_fm2400_state_Q[PRE_FM2400_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 hilbert_fm2400;
arm_fir_instance_f32 delay_fm2400;
arm_fir_instance_f32 pre_fm2400_I;
arm_fir_instance_f32 pre_fm2400_Q;

// -------------------- Post-FM-demod / TX audio LPF --------------------
#define AUDIO_TAPS 63

static float audio_lpf_coeffs[AUDIO_TAPS] = {
    // 3 kHz LPF, 48 kHz fs, Hamming window
    -0.0007, -0.0010, -0.0013, -0.0016, -0.0019, -0.0022,
    -0.0025, -0.0028, -0.0031, -0.0034, -0.0037, -0.0040,
    -0.0043, -0.0046, -0.0049, -0.0052, -0.0055, -0.0058,
    -0.0061, -0.0064, -0.0067, -0.0070, -0.0073, -0.0076,
    -0.0079, -0.0082, -0.0085, -0.0088, -0.0091, -0.0094,
    0.9900,
    -0.0094, -0.0091, -0.0088, -0.0085, -0.0082, -0.0079,
    -0.0076, -0.0073, -0.0070, -0.0067, -0.0064, -0.0061,
    -0.0058, -0.0055, -0.0052, -0.0049, -0.0046, -0.0043,
    -0.0040, -0.0037, -0.0034, -0.0031, -0.0028, -0.0025,
    -0.0022, -0.0019, -0.0016, -0.0013, -0.0010, -0.0007
};

static float audio_state[AUDIO_TAPS + BLOCK_SIZE_ANALOG/2];
arm_fir_instance_f32 audio_lpf;

// -------------------- CW receive filter --------------------
/*
 * Three identical biquad band-pass sections at the tone pitch.
 *
 * Biquads rather than a FIR because pitch and bandwidth are meant to be a
 * front panel control. Retuning this is five coefficients; a FIR with the
 * same skirts would be hundreds of taps to redesign on every turn of a knob.
 *
 * Each section is deliberately wider than the target: cascading three
 * narrows the result, so Q 1.42 per section gives about 250 Hz overall.
 * Going much below that starts to ring and smears the elements at speed,
 * which reads worse than a wider filter.
 */
#define CW_FILTER_Q  1.42f
#define CW_STAGES    3

float cw_pitch_hz = CW_PITCH_DEFAULT_HZ;

// CMSIS's biquad init stores the pointer it's given rather than copying, so
// this has to be static storage, not a stack buffer computed on the fly.
static float cw_coeffs_runtime[5 * CW_STAGES];

arm_biquad_casd_df1_inst_f32 cw_bpf;
static float cw_state[4 * CW_STAGES];

/*
 * (Re)design the CW band-pass filter for a new centre frequency, RBJ
 * "constant skirt gain" band-pass biquad, reverse-derived to match this
 * project's original fixed-700 Hz coefficients (Q 1.42, confirmed by
 * working backward from them to within rounding). Same 5 coefficients in
 * all three cascaded stages, same as the fixed table this replaces.
 */
static void cw_bpf_retune(float pitch_hz)
{
    float w0    = TWO_PI * pitch_hz / 48000.0f;
    float alpha = sinf(w0) / (2.0f * CW_FILTER_Q);
    float a0    = 1.0f + alpha;
    float cosw0 = cosf(w0);

    float b0 =  alpha / a0;
    float b2 = -alpha / a0;
    float a1 =  (2.0f * cosw0) / a0;   /* CMSIS df1 sign convention */
    float a2 = -(1.0f - alpha) / a0;

    for (int s = 0; s < CW_STAGES; s++)
    {
        cw_coeffs_runtime[5*s + 0] = b0;
        cw_coeffs_runtime[5*s + 1] = 0.0f;
        cw_coeffs_runtime[5*s + 2] = b2;
        cw_coeffs_runtime[5*s + 3] = a1;
        cw_coeffs_runtime[5*s + 4] = a2;
    }

    arm_biquad_cascade_df1_init_f32(&cw_bpf, CW_STAGES, cw_coeffs_runtime, cw_state);
}

/* Clamp and apply a new CW pitch - called from the HMI (Function -> CW
   pitch -> encoder, via IPC on H7 since the HMI is on CM4) and available
   for the console/CAT later if wanted. Retunes immediately regardless of
   MODE; harmless when not in CW, since cw_bpf just sits unused until
   MODE_CW reads it. */
void cw_set_pitch(float hz)
{
    if (hz < CW_PITCH_MIN_HZ) hz = CW_PITCH_MIN_HZ;
    if (hz > CW_PITCH_MAX_HZ) hz = CW_PITCH_MAX_HZ;
    cw_pitch_hz = hz;
    cw_bpf_retune(cw_pitch_hz);
}

/* ------------------------------------------------------------------------
 * SSB transmit filters: Hilbert (301 taps, generated here rather than a
 * static table - see ssb_tx_init()) + speech band limiting biquad.
 *
 * The Hilbert is 301 taps, which looks excessive until you notice that
 * 300 Hz is 0.6% of Nyquist at 48 kHz, and a Hilbert transformer is at its
 * worst near DC. Fewer taps cost real opposite-sideband suppression: 129
 * taps manage only about 18 dB, 201 give 30 dB, 301 give 56 dB.
 * Transmitting the unwanted sideband is other people's problem as much as
 * ours, so it is worth the arithmetic - and transmit does not run at the
 * same time as receive, so the whole CPU budget is free anyway.
 * --------------------------------------------------------------------- */
#define SSB_HIL_TAPS  301
#define SSB_HIL_DELAY ((SSB_HIL_TAPS - 1) / 2)
#define SSB_HIL_MAXBLK (BLOCK_SIZE_ANALOG / 2)

static float ssb_hil_coeffs[SSB_HIL_TAPS];
/* State is sized for the analog block, so anything longer must be fed
   through in pieces: CMSIS writes numTaps + blockSize - 1 floats here, and
   the FreeDV path calls this with 1920 where the analog modes use 256. */
static float ssb_hil_state[SSB_HIL_TAPS + SSB_HIL_MAXBLK];
static arm_fir_instance_f32 ssb_hil;

/* Speech band limiting: high pass at 300 Hz, low pass at 2700 Hz. */
static const float ssb_audio_coeffs[10] = {
    +9.7260993065e-01f,
    -1.9452198613e+00f,
    +9.7260993065e-01f,
    +1.9444697251e+00f,
    -9.4596999747e-01f,
    +2.4827170061e-02f,
    +4.9654340121e-02f,
    +2.4827170061e-02f,
    +1.5074026397e+00f,
    -6.0671131993e-01f
};
static float ssb_audio_state[8];
arm_biquad_casd_df1_inst_f32 ssb_audio;

/* Delay line for the I branch, matching the Hilbert group delay. */
static float ssb_delay[SSB_HIL_DELAY];
static int   ssb_delay_pos;

float mic_gain = 1.0f;

float ssb_osc_re = 1.0f, ssb_osc_im = 0.0f;
float ssb_step_re, ssb_step_im;

static void ssb_tx_init(void)
{
    const int M = SSB_HIL_DELAY;

    for (int i = 0; i < SSB_HIL_TAPS; i++)
    {
        int n = i - M;

        if (n == 0 || (n % 2) == 0)
        {
            ssb_hil_coeffs[i] = 0.0f;   /* a Hilbert has no even-index taps */
            continue;
        }

        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)i
                                       / (float)(SSB_HIL_TAPS - 1));

        ssb_hil_coeffs[i] = 2.0f / ((float)M_PI * (float)n) * w;
    }

    arm_fir_init_f32(&ssb_hil, SSB_HIL_TAPS, ssb_hil_coeffs,
                     ssb_hil_state, SSB_HIL_MAXBLK);
    arm_biquad_cascade_df1_init_f32(&ssb_audio, 2,
                                    (float *)ssb_audio_coeffs, ssb_audio_state);
}

void ssb_tx_restart(void)
{
    float dphi = 2.0f * (float)M_PI * 12000.0f / 48000.0f;

    ssb_step_re = cosf(dphi);
    ssb_step_im = sinf(dphi);
    ssb_osc_re  = 1.0f;
    ssb_osc_im  = 0.0f;

    for (int i = 0; i < SSB_HIL_DELAY; i++) ssb_delay[i] = 0.0f;
    ssb_delay_pos = 0;
}

/*
 * Run the Hilbert in chunks no larger than the state was sized for. The
 * filter keeps its own history between calls, so splitting the block
 * changes nothing about the output -- it only keeps CMSIS from writing
 * past the state buffer.
 */
void ssb_hilbert(const float *in, float *out, int n)
{
    for (int off = 0; off < n; off += SSB_HIL_MAXBLK)
    {
        int m = n - off;

        if (m > SSB_HIL_MAXBLK) m = SSB_HIL_MAXBLK;

        arm_fir_f32(&ssb_hil, (float *)&in[off], &out[off], m);
    }
}

/*
 * Modulate a ready audio buffer. Exposed (not static) so the FreeDV TX
 * path can reach it: a modem waveform must not be run through the speech
 * band pass or the microphone gain, since it is already shaped and any
 * further filtering or compression distorts it.
 */
void ssb_modulate(const float *audio, const float *q,
                  uint16_t *out, int n, int lsb)
{
    for (int i = 0; i < n; i++)
    {
        float d = ssb_delay[ssb_delay_pos];

        ssb_delay[ssb_delay_pos] = audio[i];
        if (++ssb_delay_pos >= SSB_HIL_DELAY) ssb_delay_pos = 0;

        float s = lsb ? (d * ssb_osc_re + q[i] * ssb_osc_im)
                      : (d * ssb_osc_re - q[i] * ssb_osc_im);

        float nre = ssb_osc_re * ssb_step_re - ssb_osc_im * ssb_step_im;
        float nim = ssb_osc_re * ssb_step_im + ssb_osc_im * ssb_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        ssb_osc_re = nre * g;
        ssb_osc_im = nim * g;

        float y = s * 1800.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

/* -------------------------------------------------------------------- */

static void nbfm_init(float if_freq_hz, float fs_hz)
{
    // NCO for IF-mixing
    nco_init(&nco_if, if_freq_hz, fs_hz);

    // Pre-demod IF-filter (shared I/Q coefficients)
    arm_fir_init_f32(&pre_fm_I,
                     PRE_DEMOD_TAPS,
                     pre_demod_coeffs,
                     pre_fm_state_I,
                     BLOCK_SIZE_MAX/2);

    arm_fir_init_f32(&pre_fm_Q,
                     PRE_DEMOD_TAPS,
                     pre_demod_coeffs,
                     pre_fm_state_Q,
                     BLOCK_SIZE_MAX/2);

    // Audio LPF after FM demod
    arm_fir_init_f32(&audio_lpf,
                     AUDIO_TAPS,
                     audio_lpf_coeffs,
                     audio_state,
                     BLOCK_SIZE_ANALOG/2);
}

/*
 * Filters are (re)initialised here rather than inline at each call site so
 * a mode change/PTT/audio-source switch can flush them all in one call.
 * FIR instances are sized for BLOCK_SIZE_MAX; calling them later with a
 * smaller block is safe, the state buffer is simply bigger than that call
 * needs.
 */
void dsp_filters_init(void)
{
    // The RX chains mix with 'nco', not the 'nco_if' that nbfm_init() sets
    // up, so it needs its own init or dphi stays 0 and nothing is
    // downconverted.
    nco_init(&nco, 12000.0f, 48000.0f);

    // NCO for the FM path, plus pre_fm_I/Q and the post-demod audio LPF.
    nbfm_init(12000.0f, 48000.0f);

    ssb_tx_init();

    cw_bpf_retune(cw_pitch_hz);   /* re-apply the current pitch, not the 700 Hz default */

    // Hilbert for SSB/NBFM RX
    arm_fir_init_f32(&hilbert,
                     HILBERT_TAPS,
                     hilbert_coeffs,
                     hilbert_state,
                     BLOCK_SIZE_MAX/2);

    /* FreeDV 2400B's own wider front-end - see the block comment above
       hilbert_fm2400_coeffs for why it can't share the two above. */
    arm_fir_init_f32(&hilbert_fm2400,
                     HILB_FM2400_TAPS,
                     hilbert_fm2400_coeffs,
                     hilbert_fm2400_state,
                     BLOCK_SIZE_MAX/2);

    arm_fir_init_f32(&delay_fm2400,
                     HILB_FM2400_TAPS,
                     delay_fm2400_coeffs,
                     delay_fm2400_state,
                     BLOCK_SIZE_MAX/2);

    arm_fir_init_f32(&pre_fm2400_I,
                     PRE_FM2400_TAPS,
                     pre_fm2400_coeffs,
                     pre_fm2400_state_I,
                     BLOCK_SIZE_MAX/2);

    arm_fir_init_f32(&pre_fm2400_Q,
                     PRE_FM2400_TAPS,
                     pre_fm2400_coeffs,
                     pre_fm2400_state_Q,
                     BLOCK_SIZE_MAX/2);
}
