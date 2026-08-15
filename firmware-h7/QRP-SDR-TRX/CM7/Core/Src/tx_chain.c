#include "tx_chain.h"
#include "modes.h"
#include "filters.h"
#include "cw_paddle.h"
#include "freedv_chain.h"
#include "stm32h7xx.h"
#include "arm_math.h"
#include <math.h>

#define TWO_PI 6.28318530718f

/* Shared scratch pool and telemetry - owned by main.c, see rx_chain.c's
   copy of this comment for why. */
#define SCRATCH_N (BLOCK_SIZE_MAX / 2)
extern float dsp_scratch[10 * SCRATCH_N];

extern volatile uint32_t rx_blocks;
extern volatile float    rx_peak;
extern volatile float    rx_adc_peak;

/* radio_tx_on()/radio_tx_off() stay in main.c - they touch HAL GPIO (LD3),
   USB Audio class resets and DMA restart, all main.c-owned resources.
   cw_keyer_poll() below starts/stops its own TX session through them,
   same as the console/CAT "tx"/"rx" commands and the HMI's PTT input. */
extern int  radio_tx_on(void);
extern void radio_tx_off(void);
extern volatile int tx_active;

/* ------------------------------------------------------------------------
 * SSB transmit dispatch (Hilbert/oscillator/modulate live in filters.c -
 * see ssb_tx_restart()/ssb_hilbert()/ssb_modulate() there)
 * --------------------------------------------------------------------- */
void ssb_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float *q     = &dsp_scratch[1 * SCRATCH_N];

    float peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio[i] = x * mic_gain;
    }

    rx_adc_peak = peak;                 /* raw mic level, before the gain */
    rx_blocks++;

    arm_biquad_cascade_df1_f32(&ssb_audio, audio, audio, n);
    ssb_hilbert(audio, q, n);

    /* USB/LSB swapped vs the phasing convention so the label matches the air,
       confirmed against an FT-857D. FreeDV keeps the plain USB call below. */
    ssb_modulate(audio, q, out, n, MODE == MODE_USB);
}

/* ------------------------------------------------------------------------
 * NBFM transmit
 * --------------------------------------------------------------------- */

/*
 * Frequency modulate a ready audio buffer.
 *
 * Deviation is a parameter because voice and data want different values:
 * 5 kHz for speech, but 2.5 kHz for FreeDV 2400B, which is what its
 * demodulator expects and what stops the discriminator running into
 * atan2's wrap point at the far end.
 */
static void fm_modulate(const float *audio, uint16_t *out, int n, float dev_hz)
{
    static float phase = 0.0f;

    const float kf   = 2.0f * (float)M_PI * dev_hz  / 48000.0f;
    const float w_if = 2.0f * (float)M_PI * 12000.0f / 48000.0f;

    for (int i = 0; i < n; i++)
    {
        phase += w_if + kf * audio[i];

        if (phase >  (float)M_PI) phase -= TWO_PI;
        if (phase < -(float)M_PI) phase += TWO_PI;

        out[i] = (uint16_t)((cosf(phase) * 2048.0f) + 2048.0f);
    }
}

void nbfm_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    /* Shared with the receive path: transmit and receive never run together. */
    float *audio_in       = &dsp_scratch[0 * SCRATCH_N];
    float *audio_filtered = &dsp_scratch[1 * SCRATCH_N];

    const float dev_hz = 5000.0f;

    float peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio_in[i] = x * mic_gain;
    }

    rx_adc_peak = peak;

    arm_fir_f32(&audio_lpf, audio_in, audio_filtered, n);

    float dpk = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio_filtered[i]);
        if (a > dpk) dpk = a;
    }

    fm_modulate(audio_filtered, out, n, dev_hz);

    rx_peak = dpk * dev_hz / 1000.0f;   /* peak deviation in kHz */
    rx_blocks++;
}

/* ------------------------------------------------------------------------
 * AM transmit
 *
 * Amplitude modulation is a carrier plus the audio riding on it: the
 * envelope is (1 + m*audio) and that multiplies a cos at the IF. Unlike
 * SSB there is no Hilbert and no sideband selection -- both sidebands are
 * sent, symmetric about the carrier -- so this is simpler than the SSB
 * modulator, not harder.
 *
 * Reads ssb_osc_re/ssb_step_re/ssb_step_im directly as its carrier rather
 * than keeping its own oscillator - see filters.h for why that's safe
 * (SSB/AM TX are never active in the same block).
 *
 * The modulation index m is kept a little under 1 so the envelope never
 * reaches zero; at m = 1 it just touches zero (100%), and above that it
 * would go negative and distort. Mic gain sets the drive into it.
 * --------------------------------------------------------------------- */
volatile int am_testtone;

void am_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];

    float peak = 0.0f;

    if (am_testtone)
    {
        /* Fixed internal 1 kHz tone at 50% depth -- removes the mic
           entirely, so a clean AM signal here proves the modulator. */
        static float ph = 0.0f;
        const float dph = 2.0f * (float)M_PI * 1000.0f / 48000.0f;

        for (int i = 0; i < n; i++)
        {
            audio[i] = 0.85f * cosf(ph);   /* ~80% modulation depth */
            ph += dph;
            if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
        }
        rx_adc_peak = 0.85f;
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            float x = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

            float a = fabsf(x);
            if (a > peak) peak = a;

            audio[i] = x * mic_gain;
        }

        rx_adc_peak = peak;

        /* Speech band limiting, reusing the SSB audio filter. */
        arm_biquad_cascade_df1_f32(&ssb_audio, audio, audio, n);
    }

    rx_blocks++;

    const float m = 0.95f;      /* keep the envelope non-negative */

    float mpk = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float env = 1.0f + m * audio[i];

        if (env < 0.0f) env = 0.0f;              /* no over-modulation */
        if (env > mpk)  mpk  = env;

        float carrier = ssb_osc_re;              /* cos at the IF */

        float nre = ssb_osc_re * ssb_step_re - ssb_osc_im * ssb_step_im;
        float nim = ssb_osc_re * ssb_step_im + ssb_osc_im * ssb_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        ssb_osc_re = nre * g;
        ssb_osc_im = nim * g;

        /* 1000 is about the ceiling: full modulation takes the envelope
           to ~2x the carrier, so 2 * 1000 fits the 2048 DAC half-swing. */
        float y = env * carrier * 1000.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }

    rx_peak = mpk;                                /* peak envelope */
}

/* ------------------------------------------------------------------------
 * CW transmit
 *
 * Sends a beacon at the IF the receiver is tuned to. The carrier sits at
 * cw_pitch_hz above the IF centre, which is the same offset the receive
 * filter is centred on: a station whose carrier gives us a 700 Hz tone is
 * on the frequency we have to answer on, so transmit has to land in the
 * same place.
 *
 * The envelope is shaped rather than switched. Hard keying splatters well
 * outside the occupied bandwidth -- key clicks are one of the more common
 * complaints on the CW bands -- so each edge is a raised cosine a few
 * milliseconds long.
 * --------------------------------------------------------------------- */

#define CW_MSG_DEFAULT "LA7LKA DE LB2S LB2S LB2S"
#define CW_MSG_MAXLEN  40                   /* text chars; cw_build_message()
                                                already bounds-checks against
                                                CW_UNITS below, so an
                                                over-length message just gets
                                                truncated safely, not overrun */
#define CW_UNITS    256                     /* dit units in the keyed message */
#define CW_RAMP     240                     /* 5 ms edge at 48 kHz */

static const char *cw_morse(char c)
{
    switch (c) {
    case 'A': return ".-";    case 'B': return "-...";  case 'C': return "-.-.";
    case 'D': return "-..";   case 'E': return ".";     case 'F': return "..-.";
    case 'G': return "--.";   case 'H': return "....";  case 'I': return "..";
    case 'J': return ".---";  case 'K': return "-.-";   case 'L': return ".-..";
    case 'M': return "--";    case 'N': return "-.";    case 'O': return "---";
    case 'P': return ".--.";  case 'Q': return "--.-";  case 'R': return ".-.";
    case 'S': return "...";   case 'T': return "-";     case 'U': return "..-";
    case 'V': return "...-";  case 'W': return ".--";   case 'X': return "-..-";
    case 'Y': return "-.--";  case 'Z': return "--..";
    case '0': return "-----"; case '1': return ".----"; case '2': return "..---";
    case '3': return "...--"; case '4': return "....-"; case '5': return ".....";
    case '6': return "-...."; case '7': return "--..."; case '8': return "---..";
    case '9': return "----.";
    default:  return "";
    }
}

static uint8_t cw_key[CW_UNITS];    /* one entry per dit unit, 1 = key down */
static int     cw_key_len;

static int      cw_wpm = 20;
static uint32_t cw_dit_samples = 48000 * 12 / (10 * 20);

static int      cw_unit;            /* index into cw_key */
static uint32_t cw_tick;            /* samples into the current unit */
static float    cw_env;             /* shaped envelope, 0..1 */
static int      cw_ramp;            /* position within an edge */

/* CAT "KY;"-triggered one-shot send (see cw_set_message()): unlike the
   looping beacon (plain "tx"/CAT "TX;", unchanged), a KY-triggered
   message should auto-return to RX after one pass. cw_msg_wrapped is set
   by cw_tx_process_block() when cw_key[] wraps back to 0 during a
   one-shot send; cw_keyer_poll() (main-loop rate, same place PTT-style
   session control already lives) acts on it. Deliberately not checked
   during live keying (cw_tx_source_live) - the two are mutually
   exclusive in practice (KY never sets cw_tx_source_live, the live
   paddle never sets cw_msg_oneshot). */
static volatile int cw_msg_oneshot;
static volatile int cw_msg_wrapped;

static float    cw_osc_re = 1.0f, cw_osc_im = 0.0f;
static float    cw_step_re, cw_step_im;

/* Sidetone oscillator: same envelope, but at cw_pitch_hz alone (no +12000
   IF offset) since this drives the speaker directly, not an IF stage. */
static float    cw_side_re = 1.0f, cw_side_im = 0.0f;
static float    cw_side_step_re, cw_side_step_im;

/*
 * Live key/paddle input - an alternative to the beacon message above, not
 * a replacement: the console "tx"/CAT "TX;" commands still send CW_MSG as
 * before (useful for RF bench testing with no key connected), while
 * physically pressing a key/paddle starts and ends its own TX session and
 * keys live. cw_tx_process_block() below picks whichever source is active
 * per TX session (cw_tx_source_live) - both feed the exact same envelope/
 * oscillator code, since that part was already a live-updatable "want"
 * gate under the hood, just fed from cw_key[] until now.
 *
 * Menu-selectable key type (see the HMI's FOCUS_KEYTYPE, via IPC on H7),
 * matching the convention on rigs like Oystein's FT-857D: Straight key
 * needs no state machine at all (want == key down, timing is entirely the
 * operator's hand). Iambic A/B run a classic squeeze keyer: while sending
 * an element, the opposite paddle being pressed latches "memory" so it
 * alternates automatically; the two modes differ only in what happens
 * right as both paddles release - mode A stops as soon as they're up,
 * mode B still sends one queued memory element first.
 */
typedef enum { CW_KEYER_STRAIGHT = 0, CW_KEYER_IAMBIC_A, CW_KEYER_IAMBIC_B } cw_keyer_type_t;
static cw_keyer_type_t cw_keyer_type = CW_KEYER_STRAIGHT;

int cw_keyer_type_get(void) { return (int)cw_keyer_type; }

void cw_keyer_type_set(int t)
{
    if (t >= CW_KEYER_STRAIGHT && t <= CW_KEYER_IAMBIC_B)
        cw_keyer_type = (cw_keyer_type_t)t;
}

const char *cw_keyer_type_name(void)
{
    static const char *names[] = { "STRAIGHT", "IAMBIC-A", "IAMBIC-B" };
    return names[cw_keyer_type];
}

/* 1 when the current TX session was started by the physical key/paddle
   (cw_keyer_poll(), below) rather than by the console/CAT beacon TX
   command - tells cw_tx_process_block() which source feeds "want". */
static int cw_tx_source_live;

typedef enum { CW_KEYER_IDLE, CW_KEYER_SEND_DIT, CW_KEYER_SEND_DAH, CW_KEYER_SPACE } cw_keyer_state_t;
static cw_keyer_state_t cw_keyer_state;
static uint32_t cw_keyer_timer;
static uint8_t  cw_keyer_dit_mem;
static uint8_t  cw_keyer_dah_mem;
static uint8_t  cw_keyer_last_was_dit;

/* Sample-rate (called once per audio sample from cw_tx_process_block(),
   same cadence as the cw_key[] array lookup it replaces) - returns the
   live "want" gate: 1 = key down / element being sent, 0 = up/space. */
static int cw_keyer_tick(void)
{
    uint8_t dit = cw_paddle_dit_read();
    uint8_t dah = (cw_keyer_type != CW_KEYER_STRAIGHT) ? cw_paddle_dah_read() : 0U;

    if (cw_keyer_type == CW_KEYER_STRAIGHT)
        return dit;

    switch (cw_keyer_state)
    {
    case CW_KEYER_IDLE:
        cw_keyer_dit_mem = 0;
        cw_keyer_dah_mem = 0;
        if (dit)
        {
            cw_keyer_state = CW_KEYER_SEND_DIT;
            cw_keyer_timer = cw_dit_samples;
            cw_keyer_last_was_dit = 1;
        }
        else if (dah)
        {
            cw_keyer_state = CW_KEYER_SEND_DAH;
            cw_keyer_timer = cw_dit_samples * 3U;
            cw_keyer_last_was_dit = 0;
        }
        return 0;

    case CW_KEYER_SEND_DIT:
    case CW_KEYER_SEND_DAH:
        if (dit) cw_keyer_dit_mem = 1;
        if (dah) cw_keyer_dah_mem = 1;
        if (--cw_keyer_timer == 0U)
        {
            cw_keyer_state = CW_KEYER_SPACE;
            cw_keyer_timer = cw_dit_samples;   /* one dit of inter-element space */
        }
        return 1;

    case CW_KEYER_SPACE:
    default:
        if (dit) cw_keyer_dit_mem = 1;
        if (dah) cw_keyer_dah_mem = 1;
        if (--cw_keyer_timer != 0U)
            return 0;

        if (!dit && !dah && cw_keyer_type == CW_KEYER_IAMBIC_A)
        {
            /* Mode A: no bonus element once both paddles are actually up. */
            cw_keyer_state = CW_KEYER_IDLE;
            return 0;
        }

        /* Alternate first (squeeze, or - mode B only - one bonus element
           right after release via memory); otherwise repeat whichever
           paddle is still physically held; otherwise stop. */
        if (!cw_keyer_last_was_dit && (dit || cw_keyer_dit_mem))
        {
            cw_keyer_state = CW_KEYER_SEND_DIT;
            cw_keyer_timer = cw_dit_samples;
            cw_keyer_last_was_dit = 1;
        }
        else if (cw_keyer_last_was_dit && (dah || cw_keyer_dah_mem))
        {
            cw_keyer_state = CW_KEYER_SEND_DAH;
            cw_keyer_timer = cw_dit_samples * 3U;
            cw_keyer_last_was_dit = 0;
        }
        else if (dit)
        {
            cw_keyer_state = CW_KEYER_SEND_DIT;
            cw_keyer_timer = cw_dit_samples;
            cw_keyer_last_was_dit = 1;
        }
        else if (dah)
        {
            cw_keyer_state = CW_KEYER_SEND_DAH;
            cw_keyer_timer = cw_dit_samples * 3U;
            cw_keyer_last_was_dit = 0;
        }
        else
        {
            cw_keyer_state = CW_KEYER_IDLE;
        }

        if (cw_keyer_state != CW_KEYER_IDLE)
        {
            cw_keyer_dit_mem = 0;
            cw_keyer_dah_mem = 0;
        }
        return 0;
    }
}

/*
 * Message text: fixed CW_MSG_DEFAULT beacon, sent on plain "tx"/CAT "TX;",
 * loops forever unless changed. Also settable at runtime via the CAT
 * "KY;" command (real Kenwood's actual "send this text now via the
 * keyer" command) and Hamlib's send_morse() hook. Setting via KY;
 * additionally triggers a ONE-SHOT send (cw_msg_oneshot below) that
 * auto-returns to RX after one pass, unlike the looping beacon - see
 * cw_keyer_poll()'s completion check.
 */
static char cw_msg[CW_MSG_MAXLEN + 1] = CW_MSG_DEFAULT;

void cw_set_message(const char *text)
{
    size_t i;

    for (i = 0; i < CW_MSG_MAXLEN && text[i]; i++)
    {
        char c = text[i];
        if (c >= 'a' && c <= 'z') c -= 32;  /* cw_morse()'s table is A-Z only */
        cw_msg[i] = c;
    }
    cw_msg[i] = 0;

    cw_key_len = 0;   /* force cw_build_message() to rebuild from the new
                          text - it's normally built once, lazily, and
                          cached forever (see cw_tx_restart()) */
}

const char *cw_get_message(void) { return cw_msg; }

static void cw_build_message(void)
{
    int n = 0;

    for (const char *p = cw_msg; *p; p++)
    {
        if (*p == ' ')
        {
            /* Word gap is 7 units; 3 were already emitted after the letter. */
            for (int i = 0; i < 4 && n < CW_UNITS; i++) cw_key[n++] = 0;
            continue;
        }

        for (const char *e = cw_morse(*p); *e; e++)
        {
            int len = (*e == '-') ? 3 : 1;

            for (int i = 0; i < len && n < CW_UNITS; i++) cw_key[n++] = 1;
            if (n < CW_UNITS) cw_key[n++] = 0;          /* inter-element gap */
        }

        for (int i = 0; i < 2 && n < CW_UNITS; i++) cw_key[n++] = 0;  /* -> 3 */
    }

    for (int i = 0; i < 7 && n < CW_UNITS; i++) cw_key[n++] = 0;      /* tail */

    cw_key_len = n;
}

void cw_set_wpm(int wpm)
{
    cw_wpm = wpm;
    cw_dit_samples = (uint32_t)(48000.0f * 1.2f / (float)wpm);
}

int cw_get_wpm(void) { return cw_wpm; }

void cw_tx_restart(void)
{
    if (!cw_key_len) cw_build_message();

    cw_unit = 0;
    cw_tick = 0;
    cw_env  = 0.0f;
    cw_ramp = 0;

    /* Carrier at the IF centre plus the tone pitch, same as receive
       expects. Runtime cw_pitch_hz (filters.h), so a pitch change moves
       the TX carrier and the RX filter together. */
    float dphi = 2.0f * (float)M_PI * (12000.0f + cw_pitch_hz) / 48000.0f;

    cw_step_re = cosf(dphi);
    cw_step_im = sinf(dphi);
    cw_osc_re  = 1.0f;
    cw_osc_im  = 0.0f;

    float dphi_side = 2.0f * (float)M_PI * cw_pitch_hz / 48000.0f;
    cw_side_step_re = cosf(dphi_side);
    cw_side_step_im = sinf(dphi_side);
    cw_side_re = 1.0f;
    cw_side_im = 0.0f;
}

void cw_tx_process_block(uint16_t *out, uint16_t *side_out, int n)
{
    for (int i = 0; i < n; i++)
    {
        /* Live key/paddle session: want comes from the real-time keyer,
           sampled fresh every sample - the beacon array below still
           advances underneath it (harmless; cw_tx_restart() always resets
           it at the start of the next session regardless of source) but
           isn't what's actually keying the carrier. */
        int want = cw_tx_source_live ? cw_keyer_tick() : cw_key[cw_unit];

        if (++cw_tick >= cw_dit_samples)
        {
            cw_tick = 0;
            if (++cw_unit >= cw_key_len)
            {
                cw_unit = 0;
                if (!cw_tx_source_live && cw_msg_oneshot) cw_msg_wrapped = 1;
            }
        }

        /* Raised-cosine edge, so the spectrum stays where it belongs. */
        if (want && cw_ramp < CW_RAMP) cw_ramp++;
        else if (!want && cw_ramp > 0) cw_ramp--;

        cw_env = 0.5f * (1.0f - cosf((float)M_PI * (float)cw_ramp / (float)CW_RAMP));

        float y = cw_env * cw_osc_re * 1800.0f + 2048.0f;

        float nre = cw_osc_re * cw_step_re - cw_osc_im * cw_step_im;
        float nim = cw_osc_re * cw_step_im + cw_osc_im * cw_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        cw_osc_re = nre * g;
        cw_osc_im = nim * g;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;

        /* Same envelope, same edge timing, just at the audio pitch instead
           of the IF carrier - so sidetone keying matches the actual
           transmitted envelope exactly, not a separately-timed copy. */
        float ys = cw_env * cw_side_re * 1800.0f + 2048.0f;

        float snre = cw_side_re * cw_side_step_re - cw_side_im * cw_side_step_im;
        float snim = cw_side_re * cw_side_step_im + cw_side_im * cw_side_step_re;
        float sg   = 1.5f - 0.5f * (snre * snre + snim * snim);

        cw_side_re = snre * sg;
        cw_side_im = snim * sg;

        if (ys < 0.0f)    ys = 0.0f;
        if (ys > 4095.0f) ys = 4095.0f;

        side_out[i] = (uint16_t)ys;
    }
}

/*
 * Main-loop-rate (not sample-rate) polling for the physical key/paddle -
 * decides when a live-keyed TX session should start and end, the same
 * coarse "session" role PTT plays for voice modes. Deliberately NOT
 * sample-rate: radio_tx_on()/radio_tx_off() reset DMA/USB audio rings and
 * are far too expensive to call more than once per over. The actual
 * element-by-element keying happens separately, at sample rate, in
 * cw_keyer_tick() inside cw_tx_process_block() above.
 */
/* How long the key/paddle can sit idle mid-session before TX drops.
   MUST scale with WPM, not be a fixed duration: a normal inter-word gap
   is 7 dit-units, which at the default 20 WPM alone is already 420 ms -
   a fixed 300 ms dropped TX, and paid the full radio_tx_on()/DMA-reset
   cost, between every single word. 10 dit units gives clear margin over
   the 7-unit word gap at any speed, while still ending a genuinely-
   finished over reasonably promptly - and because it's dit-unit-based,
   it stays correct automatically as WPM (cw_dit_samples) changes. */
#define CW_KEYER_HANG_DIT_UNITS 10U

void cw_keyer_poll(void)
{
    static uint32_t last_active_tick;

    if (MODE != MODE_CW)
        return;

    /* KY-triggered one-shot message finished its one pass - auto-return
       to RX, unlike the looping beacon. Checked here since this already
       runs every main-loop iteration while in CW mode. */
    if (cw_msg_oneshot && cw_msg_wrapped)
    {
        radio_tx_off();
    }

    uint8_t active = cw_paddle_dit_read()
                    || (cw_keyer_type != CW_KEYER_STRAIGHT && cw_paddle_dah_read());
    uint32_t now = HAL_GetTick();
    uint32_t hang_ms = (cw_dit_samples * CW_KEYER_HANG_DIT_UNITS) / 48U;  /* samples -> ms at 48 kHz */

    if (active)
    {
        last_active_tick = now;
        if (!tx_active)
        {
            cw_tx_source_live = 1;
            radio_tx_on();
        }
    }
    else if (tx_active && cw_tx_source_live && (now - last_active_tick) >= hang_ms)
    {
        radio_tx_off();   /* clears cw_tx_source_live itself */
    }
}

/* ------------------------------------------------------------------------
 * FreeDV transmit.
 *
 * The microphone goes into codec2, and what comes back is a modem waveform
 * that still has to be put on the air by one of the analog modulators:
 * 1600, 700D and 700E ride on SSB, 2400B on FM. So this is the vocoder and
 * modem in front of the modulators, not a modulator of its own.
 *
 * Note what is deliberately absent: no speech band pass, no microphone
 * gain, no compression on the modem waveform. Those belong to voice.
 * Applying them to a modem signal distorts the very thing the far end has
 * to demodulate, which is why FreeDV operating tells you to set drive by
 * peak and leave ALC out of it.
 * --------------------------------------------------------------------- */

extern int freedv_ok;

/*
 * Feed one block of microphone audio into the FreeDV chain (decimate only).
 * The encode is separate, so this stays cheap and can run every block.
 */
void freedv_tx_feed(const uint16_t *in, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float  peak  = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = ((float)in[i] - ADC_MID_16B) / ADC_MID_16B;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio[i] = x * mic_gain;        /* gain on speech, before codec2 */
    }

    rx_adc_peak = peak;
    rx_blocks++;

    if (freedv_ok) freedv_chain_put_speech48(audio, n);
}

/*
 * Produce one block of modem IF into the DAC. Pulls interpolated modem
 * samples and runs them through the SSB or FM modulator. Cheap -- no
 * encode here.
 */
void freedv_tx_produce(uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float *q     = &dsp_scratch[1 * SCRATCH_N];

    if (!freedv_ok)
    {
        for (int i = 0; i < n; i++) out[i] = 2048;
        return;
    }

    freedv_chain_get_modem48(audio, n);     /* modem waveform, not speech */

    float mpk = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio[i]);
        if (a > mpk) mpk = a;
    }
    rx_peak = mpk;                          /* modem drive level */

    if (MODE == MODE_FREEDV_2400B)
    {
        fm_modulate(audio, out, n, 2500.0f);   /* what 2400B expects */
    }
    else
    {
        ssb_hilbert(audio, q, n);
        ssb_modulate(audio, q, out, n, 0);  /* FreeDV rides on USB */
    }
}

/* ------------------------------------------------------------------------
 * Transmit dispatch. CW keys its own carrier and takes no input; the voice
 * modes modulate whatever is on the microphone ADC. side_out is the CW
 * sidetone buffer - only CW writes it, since it's the only TX mode DAC1
 * stays running for.
 * --------------------------------------------------------------------- */
void tx_process_block(const uint16_t *in, uint16_t *out, uint16_t *side_out, int n)
{
    if (MODE == MODE_CW)                            cw_tx_process_block(out, side_out, n);
    else if (MODE == MODE_USB || MODE == MODE_LSB)  ssb_tx_process_block(in, out, n);
    else if (MODE == MODE_NBFM)                     nbfm_tx_process_block(in, out, n);
    else if (MODE == MODE_AM)                       am_tx_process_block(in, out, n);
    else
    {
        for (int i = 0; i < n; i++) out[i] = 2048;  /* nothing to send yet */
    }
}

int tx_supported(int mode)
{
    return mode == MODE_CW || mode == MODE_USB || mode == MODE_LSB
        || mode == MODE_NBFM || mode == MODE_AM || mode_is_freedv(mode);
}
