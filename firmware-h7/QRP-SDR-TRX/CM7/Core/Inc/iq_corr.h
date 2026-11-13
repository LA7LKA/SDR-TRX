/*
 * iq_corr.h - automatic (blind) RX I/Q gain/phase/DC correction.
 *
 * The quadrature mixer and the two analog paths after it never match
 * perfectly. A gain error g and phase error phi turn every signal at +f
 * into a faint copy at -f (the image), which with the 12 kHz low IF lands
 * a station 24 kHz away inside the passband. 0.5 deg and 0.1 dB still
 * cap image rejection near 45 dB; the bench adapter's 8 dB was 7 dB.
 *
 * Estimation needs no test signal. What reaches the antenna at +f and -f
 * is uncorrelated, so a perfect receiver gives E[I^2] = E[Q^2] and
 * E[I*Q] = 0 (a "proper" complex signal). Any departure is the hardware:
 *
 *     g    = sqrt(E[Q^2] / E[I^2])               Q gain relative to I
 *     sin(phi) = E[I*Q] / sqrt(E[I^2] E[Q^2])     Q phase error
 *
 * and Q is rebuilt as (Q/g - I sin(phi)) / cos(phi). Averages run over a
 * few seconds, so the estimate tracks drift (temperature, band, gain
 * steps) but not the signal itself. DC is removed first - it would read
 * as a gain error, and at 0 Hz it's out of band at 12 kHz IF anyway.
 *
 * Only one tap - a gain/phase error that is the same across the IF band.
 * Measured on the bench that holds (0.4-0.6 deg over 3-18 kHz); a
 * frequency-dependent error would need an FIR version.
 *
 * RX only. TX needs a measured loopback, since the samples going to the
 * DACs are ideal by construction - see doc in iq_corr.c.
 */
#ifndef IQ_CORR_H
#define IQ_CORR_H

#include <stdint.h>

/* Controls, set from the debugger (no CAT on H7 yet). */
extern volatile int32_t iqc_enable;   /* 1 = correct (default), 0 = raw
                                         ADC data, as before - for
                                         measuring the converters */
extern volatile int32_t iqc_freeze;   /* 1 = keep the current correction
                                         but stop adapting */
extern volatile int32_t iqc_reset;    /* write 1: restart estimation;
                                         cleared when done */

/* Readout, updated every block. */
extern volatile float iqc_gain_db;    /* Q relative to I, as measured */
extern volatile float iqc_phase_deg;  /* Q phase error, as measured */
extern volatile float iqc_dc_i;       /* removed DC, full scale = 1.0 */
extern volatile float iqc_dc_q;
extern volatile float iqc_irr_in_db;  /* image rejection estimated on the
                                         raw input ... */
extern volatile float iqc_irr_out_db; /* ... and after correction */

void iq_corr_init(void);

/* Correct one block in place. i/q are signed, full scale +-32768;
   n samples at fs Hz. Does nothing to the data when iqc_enable == 0. */
void iq_corr_process(int16_t *iq_interleaved, uint32_t n, float fs);

#endif
