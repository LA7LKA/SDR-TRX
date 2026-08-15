#ifndef FILTERS_H
#define FILTERS_H

#include "stm32h7xx.h"
#include "arm_math.h"
#include "dsp.h"

/*
 * Filter instances, coefficients, and the shared IF-mixing NCOs, in one
 * place instead of scattered through main.c (ported from firmware/Core/Src/
 * main.c, where all of this lived inline with the RX/TX chains that use it).
 *
 * Left out on purpose: F746's main.c also declared `pre_demod_filter`
 * (arm_fir_instance_f32) and a 101-tap `if_coeffs` table - grep-verified
 * dead code, never initialized or called anywhere. The real IF pre-filter
 * is pre_fm_I/pre_fm_Q below (73-tap pre_demod_coeffs, via nbfm_init()).
 */

/* Shared downconversion NCOs - 'nco' for the SSB/CW/AM/FreeDV-on-SSB path
   (ssb_process_block() etc.), 'nco_if' for the NBFM path (nbfm_init()
   sets its frequency). Both live here since dsp_filters_init() owns them. */
extern nco_t nco;
extern nco_t nco_if;

/* Fills an interleaved cos/sin buffer from 'nco' for one block - shared by
   every RX chain function that downconverts via 'nco' (not 'nco_if',
   which nbfm_process_block() steps sample-by-sample instead). */
void nco_block_iq(float *NCO_buf, int n);

/* RX Hilbert (I -> Q), shared by ssb_process_block()/nbfm_process_block() -
   NOT am_process_block(), which downconverts directly via nco_block_iq()
   instead and needs no Hilbert, and NOT freedv2400b_process_block(), which
   has its own wider hilbert_fm2400/delay_fm2400 pair below.
   Named "hilbert" for historical reasons but it is actually a symmetric
   linear-phase bandpass (0 deg phase shift, centre tap 0.383), not a
   Hilbert transformer - image rejection comes from the post-mix lowpass. */
extern arm_fir_instance_f32 hilbert;

/* IF pre-filter (complex: I and Q run through identical coefficients),
   shared by every RX chain except freedv2400b_process_block(), and by
   nbfm_tx_process_block()/TX audio paths that need it. */
extern arm_fir_instance_f32 pre_fm_I;
extern arm_fir_instance_f32 pre_fm_Q;

/* FreeDV 2400B's own front-end. Two reasons it can't share the above:
   its FM signal is far wider (needs the +-4800 Hz sidebands that carry the
   data), and the shared path's Z = I_raw + j*(h*I_raw) construction combs
   because h delays by (taps-1)/2 and I_raw doesn't. hilbert_fm2400 is a
   real Hilbert transformer and delay_fm2400 is the matching pure delay for
   the I leg, so the pair is properly analytic. See the block comment above
   hilbert_fm2400_coeffs in filters.c. */
extern arm_fir_instance_f32 hilbert_fm2400;
extern arm_fir_instance_f32 delay_fm2400;
extern arm_fir_instance_f32 pre_fm2400_I;
extern arm_fir_instance_f32 pre_fm2400_Q;

/* Post-FM-demod / TX audio LPF - 3 kHz, reused on both RX (after
   nbfm_demod()) and TX (nbfm_tx_process_block()'s mic audio). */
extern arm_fir_instance_f32 audio_lpf;

/* CW receive filter: 3-stage biquad band-pass, retuned around cw_pitch_hz.
   cw_set_pitch() is the front-panel-control entry point (HMI -> IPC ->
   here); it clamps to [CW_PITCH_MIN_HZ, CW_PITCH_MAX_HZ] and retunes
   immediately. Also sets the TX carrier offset - see modes/tx_chain, RX
   filter and TX pitch are one linked setting, not two. */
#define CW_PITCH_DEFAULT_HZ  700.0f
#define CW_PITCH_MIN_HZ      300.0f
#define CW_PITCH_MAX_HZ     1000.0f

extern float cw_pitch_hz;
extern arm_biquad_casd_df1_inst_f32 cw_bpf;

void cw_set_pitch(float hz);

/* SSB TX Hilbert (301 taps, generated at runtime in ssb_tx_init() - see
   filters.c) and speech band-limiting biquad (300-2700 Hz), shared by
   ssb_tx_process_block(), am_tx_process_block() (band-limiting only) and
   freedv_tx_produce() (Hilbert only, for the non-2400B modes riding on
   SSB). */
void ssb_hilbert(const float *in, float *out, int n);
extern arm_biquad_casd_df1_inst_f32 ssb_audio;

/*
 * SSB/AM shared IF carrier oscillator + the SSB modulator.
 *
 * am_tx_process_block() (tx_chain.c) reads ssb_osc_re/ssb_step_re/
 * ssb_step_im directly as its own carrier rather than keeping a separate
 * oscillator - safe because SSB/LSB/AM TX are never active in the same
 * block (tx_process_block() dispatches to exactly one), same "mutually
 * exclusive in time" pattern as dsp_scratch. Preserved as-is from F746
 * rather than redesigned, since the coupling is deliberate, not
 * accidental.
 */
extern float ssb_osc_re, ssb_osc_im;
extern float ssb_step_re, ssb_step_im;

void  ssb_tx_restart(void);
void  ssb_modulate(const float *audio, const float *q,
                   uint16_t *out, int n, int lsb);

/* Microphone gain (1..200), console/HMI-adjustable. Applied by every TX
   modulator that reads a real mic signal. */
extern float mic_gain;

/* ADC1 (IF RX) and ADC2 (mic) both run at 16-bit resolution
   (ADC_RESOLUTION_16B, main.c) - full range 0..65535, midpoint/scale
   32768. Every RX/TX chain function normalizes its raw ADC sample with
   this. Unrelated to DAC_MID in main.c, which stays 12-bit-scaled since
   STM32's DAC hardware is fixed 12-bit regardless of ADC resolution. */
#define ADC_MID_16B 32768.0f

/*
 * (Re)initialise every filter/NCO/oscillator above. Called on mode change,
 * PTT, and audio-source switch (see radio_set_mode()/radio_tx_on()/
 * audio_source_set() in main.c) so stale state from before never bleeds
 * into the new stream. FIR instances are sized for BLOCK_SIZE_MAX; calling
 * them later with a smaller block is safe.
 */
void dsp_filters_init(void);

#endif
