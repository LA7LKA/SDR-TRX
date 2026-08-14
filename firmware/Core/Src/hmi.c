#include "hmi.h"
#include "oled.h"
#include "encoder.h"
#include "buttons.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/*
 * Owned by main.c - the existing radio state/API this module drives.
 * mode/PTT/mic gain are real; radio_set_mode() and radio_tx_on/off() are
 * the exact same functions the UART console's "mode"/"tx"/"rx" commands
 * call, so the panel and the console can never drift apart on how the
 * radio actually gets keyed or mode-switched.
 */
extern int   MODE;
extern float mic_gain;
extern float cw_pitch_hz;
extern volatile int tx_active;
/* 0.0-1.0 full-scale RMS, computed AFTER tuning/demod (main.c's per-mode
   RX branches) - see format_smeter() below. Deliberately not rx_adc_peak:
   that one is measured before the NCO mix, on the raw broadband IF, so it
   doesn't track whatever's actually tuned in - it read a near-constant
   value regardless of signal, which is exactly the bug this replaced.
   rx_rms now covers every mode (main.c was missing it for USB/LSB/FreeDV
   until this fix, added the same day). */
extern volatile float rx_rms;
int  mode_count(void);
const char *mode_name(int mode);
void radio_set_mode(int mode);
int  radio_tx_on(void);
void radio_tx_off(void);
void cw_set_pitch(float hz);
int  audio_source_is_usb(void);
void audio_source_toggle(void);
int  cw_keyer_type_get(void);
void cw_keyer_type_set(int t);
const char *cw_keyer_type_name(void);

/* Button numbers, in D2..D10+D12 wiring order - see buttons.h. */
enum {
    BTN_FUNCTION = 1,
    BTN_MODE     = 2,
    BTN_BAND     = 3,
    BTN_RIT      = 4,
    BTN_STEP     = 5,
    BTN_FILTER   = 6,
    BTN_SPLIT    = 7,
    BTN_TUNE     = 8,
    BTN_LOCK     = 9,
    BTN_ENTER    = 10,
};

/* Nucleo USER_Btn: PC13, idle low, pressed high (its own on-board pull-down
   already exists, MX_GPIO_Init() already sets GPIO_NOPULL for it). Doubles
   as PTT on this test board only. */
#define PTT_PORT GPIOC
#define PTT_PIN  GPIO_PIN_13

typedef enum { FOCUS_FREQ, FOCUS_VOLUME, FOCUS_MIC, FOCUS_CWPITCH, FOCUS_KEYTYPE, FOCUS_POWER, FOCUS_RIT } focus_t;

static const uint32_t step_table[] = { 10, 100, 1000, 10000 };
#define STEP_COUNT ((int)(sizeof(step_table) / sizeof(step_table[0])))

static const char *filt_name[] = { "NORM", "NAR", "WIDE" };
#define FILT_COUNT ((int)(sizeof(filt_name) / sizeof(filt_name[0])))

/*
 * Skeleton band table: one representative frequency per HF amateur band.
 * There is no real LO/relay hardware yet, so BAND and FREQ below are a UI
 * placeholder Oystein asked for explicitly - they track state and draw on
 * the OLED, but do not tune anything real. MIC (mic_gain) is the one
 * exception: it drives the actual TX audio chain, since that variable
 * already existed and the console's "mic" command already adjusted it.
 *
 * All ten HF bands, not just one per BPF/LPF filter - the real front end
 * pairs some of these behind a shared filter (60+40, 30+20, 17+15, 12+10,
 * see doc/architecture.md's filter table), but the operator still picks a
 * band, not a filter, so the skeleton follows that rather than the
 * hardware grouping.
 */
static const uint32_t band_freq[] = {
    1900000, 3700000, 5300000, 7100000, 10125000,
    14200000, 18120000, 21200000, 24940000, 28400000,
};
static const char *band_name[] = {
    "160", "80", "60", "40", "30", "20", "17", "15", "12", "10",
};
#define BAND_COUNT ((int)(sizeof(band_freq) / sizeof(band_freq[0])))

static uint32_t vfo_freq_hz    = 14200000;
static int      band_idx       = 5;     /* index into band_freq/band_name - matches vfo_freq_hz above (20m) */
static int      step_idx       = 2;     /* 1 kHz */
static int      filt_idx       = 0;
static int32_t  rit_offset_hz  = 0;
static int      volume         = 50;    /* display only for now - no audio gain variable to drive yet */
static int      tx_power_pct   = 100;   /* display only for now - the PWM+VCA ALC that would set this
                                            (see rf-hardware-chain notes) doesn't exist in hardware yet */

static focus_t  focus        = FOCUS_FREQ;
static int      rit_active   = 0;
static int      split_active = 0;
static int      tune_active  = 0;       /* display only - doesn't key a real carrier yet */
static int      locked       = 0;

/* "14200000" -> "14.200.000" - thousands-grouped, for the two frequency
   lines. Groups from the right; the leftmost group is whatever's left over
   (1-3 digits), same convention as normal thousands separators. */
static void format_freq_grouped(char *buf, size_t bufsz, uint32_t hz)
{
    char digits[12];
    int  n = snprintf(digits, sizeof(digits), "%lu", (unsigned long)hz);
    int  first_group = n % 3;
    if (first_group == 0) first_group = 3;

    char *out = buf;
    char *end = buf + bufsz - 1;
    int   copied = 0;

    while (copied < n && out < end)
    {
        int group_len = (copied == 0) ? first_group : 3;
        while (group_len-- > 0 && copied < n && out < end)
            *out++ = digits[copied++];
        if (copied < n && out < end)
            *out++ = '.';
    }
    *out = 0;
}

/* Uncalibrated relative S-meter, estimated straight off the post-demod RX
   RMS (rx_rms, 0.0-1.0 full-scale) - there's no real RF front end/AGC
   hardware yet to calibrate a proper S-unit-per-uV reading against (see
   rf-hardware-chain notes), so this is a placeholder: -20 dBFS pinned to
   S9, 6 dB/S-unit below that (the standard, if not perfectly universal,
   ham convention), "+NN DB" appended once past S9. No damping/hold yet,
   so it'll move as fast as the underlying audio does - worth adding a
   decay filter later if it reads too jumpy on real hardware. Revisit
   properly once real RF AGC/S-meter hardware exists. */
static void format_smeter(char *buf, size_t bufsz)
{
    float level = rx_rms;
    if (level < 0.00003f) level = 0.00003f;   /* -90 dBFS floor, avoid log(0) */
    float dbfs = 20.0f * log10f(level);

    int s_unit, over_db = 0;
    if (dbfs >= -20.0f)
    {
        s_unit = 9;
        over_db = (int)(dbfs + 20.0f);
        if (over_db < 0) over_db = 0;
    }
    else
    {
        s_unit = 9 - (int)((-20.0f - dbfs) / 6.0f);
        if (s_unit < 1) s_unit = 1;
        if (s_unit > 9) s_unit = 9;
    }

    char bars[10];
    int  i;
    for (i = 0; i < s_unit && i < 9; i++) bars[i] = '-';
    bars[i] = 0;

    if (over_db > 0) snprintf(buf, bufsz, "S%d %s+%dDB", s_unit, bars, over_db);
    else             snprintf(buf, bufsz, "S%d %s", s_unit, bars);
}

static void redraw(void)
{
    char l_smeter[24], l_freq_sm[24], l_freq_big[16], l_audio_fn[32], l_mode[24], l_flags[40];
    char freq_grp[16];

    format_smeter(l_smeter, sizeof(l_smeter));

    format_freq_grouped(freq_grp, sizeof(freq_grp), vfo_freq_hz);
    snprintf(l_freq_sm, sizeof(l_freq_sm), "%s F:%sHZ", tx_active ? "TX" : "RX", freq_grp);
    snprintf(l_freq_big, sizeof(l_freq_big), "%s", freq_grp);

    char fn_val[16];
    switch (focus)
    {
    case FOCUS_FREQ:    snprintf(fn_val, sizeof(fn_val), "STEP %luHZ", (unsigned long)step_table[step_idx]); break;
    case FOCUS_VOLUME:  snprintf(fn_val, sizeof(fn_val), "VOL %d", volume); break;
    case FOCUS_MIC:     snprintf(fn_val, sizeof(fn_val), "MIC %d", (int)mic_gain); break;
    case FOCUS_CWPITCH: snprintf(fn_val, sizeof(fn_val), "PITCH %dHZ", (int)cw_pitch_hz); break;
    case FOCUS_KEYTYPE: snprintf(fn_val, sizeof(fn_val), "KEY %s", cw_keyer_type_name()); break;
    case FOCUS_POWER:   snprintf(fn_val, sizeof(fn_val), "PWR %d", tx_power_pct); break;
    case FOCUS_RIT:     snprintf(fn_val, sizeof(fn_val), "RIT %ldHZ", (long)rit_offset_hz); break;
    default:            fn_val[0] = 0; break;
    }
    snprintf(l_audio_fn, sizeof(l_audio_fn), "A:%s FN:%s",
             audio_source_is_usb() ? "USB" : "ANLG", fn_val);

    snprintf(l_mode, sizeof(l_mode), "M:%s %s", mode_name(MODE), filt_name[filt_idx]);

    /* Band name has nowhere else to live now that the old band/freq line is
       gone (freq lines above show frequency only) - keep it visible here,
       always shown, same as before; the other flags stay conditional. */
    snprintf(l_flags, sizeof(l_flags), "%sM ", band_name[band_idx]);
    if (locked)       strcat(l_flags, "LOCK ");
    if (rit_active)   strcat(l_flags, "RIT ");
    if (split_active) strcat(l_flags, "SPLIT ");
    if (tune_active)  strcat(l_flags, "TUNE ");

    oled_clear();
    oled_draw_text(0, 0, l_smeter);
    oled_draw_text(0, 1, l_freq_sm);
    oled_draw_text_2x(0, 2, l_freq_big);
    oled_draw_text(0, 4, l_audio_fn);
    oled_draw_text(0, 5, l_mode);
    oled_draw_text(0, 6, l_flags);
    oled_display();
}

void hmi_init(I2C_HandleTypeDef *oled_i2c)
{
    encoder_init();
    buttons_init();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = PTT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PTT_PORT, &gpio);

    oled_init(oled_i2c);
    redraw();
}

void hmi_poll(void)
{
    static int32_t  enc_last        = 0;
    static uint16_t btn_prev        = 0;
    static uint8_t  ptt_prev        = 0;
    static uint32_t oled_last_tick  = 0;
    static uint32_t btn_debounce_tick = 0;
    static uint32_t ptt_debounce_tick = 0;

    uint32_t now_tick = HAL_GetTick();

    int32_t enc_now   = encoder_poll();
    int32_t enc_delta = enc_now - enc_last;
    enc_last = enc_now;

    uint16_t btn_now  = buttons_read();
    uint16_t btn_edge = (uint16_t)(btn_now & ~btn_prev);   /* bits that just went 0->1 */
    btn_prev = btn_now;
    if (btn_edge && (now_tick - btn_debounce_tick) < 150)
        btn_edge = 0;   /* contact bounce, same physical press */
    if (btn_edge)
        btn_debounce_tick = now_tick;

    /* PTT: press-and-hold, not a toggle, so this stays level-based rather
       than edge-based like the numbered buttons. */
    uint8_t ptt_now = (HAL_GPIO_ReadPin(PTT_PORT, PTT_PIN) == GPIO_PIN_SET) ? 1 : 0;
    if (ptt_now != ptt_prev && (now_tick - ptt_debounce_tick) >= 30)
    {
        ptt_prev = ptt_now;
        ptt_debounce_tick = now_tick;
        if (ptt_now) radio_tx_on();
        else         radio_tx_off();
        oled_last_tick = now_tick;
        redraw();   /* PTT is its own state change - nothing else below sees it */
    }

    if (locked)
    {
        /* Only Lock itself gets through, so the panel can be unlocked. PTT
           above is deliberately not gated by this - a locked panel should
           not also lock you out of transmitting. */
        if (btn_edge & (1u << (BTN_LOCK - 1)))
        {
            locked = 0;
            redraw();
        }
        return;
    }

    int changed = 0;

    /* S-meter (and anything else continuously live, not just discrete
       button/encoder edges) needs its own periodic redraw trigger -
       without this, redraw() only ever fires on a UI event, so the meter
       just shows whatever rx_rms happened to be at the last button press
       or mode change and sits frozen between them. 200 ms (5 Hz) is fine
       for a meter - no need to push I2C traffic at the encoder's 20 Hz cap
       for something nobody needs video-rate updates on. */
    {
        static uint32_t meter_tick = 0;
        if ((now_tick - meter_tick) >= 200)
        {
            meter_tick = now_tick;
            changed = 1;
        }
    }

    /* tx_active and audio_source can also change from outside this module
       (the UART console's "tx"/"rx"/"src" commands) - without this check
       the OLED only ever reflected changes hmi_poll() itself caused
       (button/encoder edges), so a console-driven change left the display
       showing stale state, e.g. still "AUDIO:USB" after "src analog" was
       typed at the console. Compare against what we last drew and pick
       up any external change the same way a local edit would. */
    {
        static int last_tx_active = -1;
        static int last_audio_usb = -1;
        int now_usb = audio_source_is_usb();

        if ((int)tx_active != last_tx_active) { last_tx_active = tx_active; changed = 1; }
        if (now_usb != last_audio_usb)        { last_audio_usb = now_usb;   changed = 1; }
    }

    if (btn_edge & (1u << (BTN_FUNCTION - 1)))
    {
        focus = (focus == FOCUS_POWER) ? FOCUS_FREQ : (focus_t)(focus + 1);
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_MODE - 1)))
    {
        int next = MODE + 1;
        if (next >= mode_count()) next = 0;
        radio_set_mode(next);
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_BAND - 1)))
    {
        band_idx = (band_idx + 1) % BAND_COUNT;
        vfo_freq_hz = band_freq[band_idx];
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_RIT - 1)))
    {
        rit_active = !rit_active;
        focus = rit_active ? FOCUS_RIT : FOCUS_FREQ;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_STEP - 1)))
    {
        step_idx = (step_idx + 1) % STEP_COUNT;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_FILTER - 1)))
    {
        filt_idx = (filt_idx + 1) % FILT_COUNT;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_SPLIT - 1)))
    {
        split_active = !split_active;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_TUNE - 1)))
    {
        /* Toggled TX, same mechanism as PTT - keys whatever the current
           mode's real TX chain produces (steady carrier for AM/CW, only as
           much as the mic picks up for SSB). A dedicated tuner-peaking tone
           that ignores mode would need its own always-on carrier generator,
           not built yet. */
        tune_active = !tune_active;
        if (tune_active) radio_tx_on();
        else              radio_tx_off();
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_LOCK - 1)))
    {
        locked = 1;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_ENTER - 1)))
    {
        /* Only menu function assigned to this button so far: toggle audio
           routing between the analog ADC/DAC pins and the USB Audio class
           (see main.c's audio_source). A fuller menu system is still not
           built. */
        audio_source_toggle();
        changed = 1;
    }

    if (enc_delta != 0)
    {
        switch (focus)
        {
        case FOCUS_FREQ:
            vfo_freq_hz = (uint32_t)((int64_t)vfo_freq_hz
                          + (int64_t)enc_delta * step_table[step_idx]);
            break;
        case FOCUS_VOLUME:
            volume += enc_delta;
            if (volume < 0)   volume = 0;
            if (volume > 100) volume = 100;
            break;
        case FOCUS_MIC:
            mic_gain += (float)enc_delta;
            if (mic_gain < 1.0f)   mic_gain = 1.0f;
            if (mic_gain > 200.0f) mic_gain = 200.0f;
            break;
        case FOCUS_CWPITCH:
            /* cw_set_pitch() clamps and retunes the real RX filter (and
               moves the TX carrier the same amount) immediately. */
            cw_set_pitch(cw_pitch_hz + (float)(enc_delta * 10));
            break;
        case FOCUS_KEYTYPE:
        {
            /* A 3-item discrete menu, not an analog value - one step per
               poll interval regardless of how far the encoder actually
               turned, same reasoning FOCUS_MODE-style button cycling
               already uses (a fast turn shouldn't skip through choices
               unpredictably). */
            int kt = cw_keyer_type_get() + ((enc_delta > 0) ? 1 : -1);
            if (kt < 0) kt = 2;
            if (kt > 2) kt = 0;
            cw_keyer_type_set(kt);
            break;
        }
        case FOCUS_POWER:
            tx_power_pct += enc_delta * 5;
            if (tx_power_pct < 0)   tx_power_pct = 0;
            if (tx_power_pct > 100) tx_power_pct = 100;
            break;
        case FOCUS_RIT:
            rit_offset_hz += enc_delta * 10;
            if (rit_offset_hz < -9990) rit_offset_hz = -9990;
            if (rit_offset_hz > 9990)  rit_offset_hz = 9990;
            break;
        }
        changed = 1;
    }

    if (changed && (now_tick - oled_last_tick) >= 50)
    {
        oled_last_tick = now_tick;
        redraw();
    }
}

uint32_t hmi_get_vfo_freq(void)
{
    return vfo_freq_hz;
}

void hmi_set_vfo_freq(uint32_t freq_hz)
{
    vfo_freq_hz = freq_hz;
    redraw();   /* called from outside hmi_poll(), same as the PTT branch above */
}
