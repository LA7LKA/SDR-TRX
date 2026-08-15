#include "hmi.h"
#include "oled.h"
#include "encoder.h"
#include "buttons.h"
#include "modes.h"
#include "ipc_mailbox.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/*
 * Everything that used to be "extern int MODE" / a direct radio_set_mode()
 * call on F746 (same core, same address space) is now local desired-state
 * here, pushed to CM7 through the IPC mailbox (ipc_mailbox.h) instead -
 * see ipc_push() below. rx_rms/tx_active/the confirmed mode come back the
 * same way, in ipc_cm7_to_cm4_read()'s status struct.
 */

/* CW pitch range - must match filters.h's CW_PITCH_MIN_HZ/MAX_HZ on CM7;
   duplicated here since CM4 has no reason to pull in all of filters.h. */
#define CW_PITCH_DEFAULT_HZ  700.0f
#define CW_PITCH_MIN_HZ      300.0f
#define CW_PITCH_MAX_HZ     1000.0f

/* Button numbers, in D2,D3,D4,D5,D9,D10,D11,D14,D15,A0 wiring order - see
   buttons.h. Same logical layout as F746, different physical pins. */
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

/* Nucleo USER_Btn: PC13, idle low, pressed high. Doubles as PTT on this
   test board only, same as F746 - assign to the CortexM4 context in
   CubeMX (not yet done as of this port). */
#define PTT_PORT GPIOC
#define PTT_PIN  GPIO_PIN_13

typedef enum { FOCUS_FREQ, FOCUS_VOLUME, FOCUS_MIC, FOCUS_CWPITCH, FOCUS_KEYTYPE, FOCUS_POWER, FOCUS_RIT } focus_t;

static const uint32_t step_table[] = { 10, 100, 1000, 10000 };
#define STEP_COUNT ((int)(sizeof(step_table) / sizeof(step_table[0])))

static const char *filt_name[] = { "NORM", "NAR", "WIDE" };
#define FILT_COUNT ((int)(sizeof(filt_name) / sizeof(filt_name[0])))

static const char *keyer_type_name[] = { "STRAIGHT", "IAMBIC-A", "IAMBIC-B" };

/* Skeleton band table: UI placeholder, no real LO/relay hardware yet -
   same as F746 (see that file's own comment for the full reasoning). */
static const uint32_t band_freq[] = {
    1900000, 3700000, 5300000, 7100000, 10125000,
    14200000, 18120000, 21200000, 24940000, 28400000,
};
static const char *band_name[] = {
    "160", "80", "60", "40", "30", "20", "17", "15", "12", "10",
};
#define BAND_COUNT ((int)(sizeof(band_freq) / sizeof(band_freq[0])))

static uint32_t vfo_freq_hz    = 14200000;
static int      band_idx       = 5;
static int      step_idx       = 2;
static int      filt_idx       = 0;
static int32_t  rit_offset_hz  = 0;
static int      volume         = 50;
static int      tx_power_pct   = 100;

static focus_t  focus        = FOCUS_FREQ;
static int      rit_active   = 0;
static int      split_active = 0;
static int      tune_active  = 0;
static int      locked       = 0;

/* Desired state pushed to CM7 - mirrors ipc_cm4_to_cm7_t. mode starts at
   MODE_USB, matching CM7's own default (modes.c: int MODE = MODE_USB). */
static int32_t desired_mode        = MODE_USB;
static float   desired_cw_pitch_hz = CW_PITCH_DEFAULT_HZ;
static int32_t desired_cw_wpm      = 20;
static int32_t desired_cw_keyer_type = 0;
static float   desired_mic_gain    = 1.0f;

/* Was a hmi_poll()-local static; promoted to file scope so hmi_set_mode()
   (console.c's "mode" command) can reuse it too, preserving the same
   "physical PTT || TUNE, latched across polls" value either caller sees. */
static uint8_t  ptt_now_level      = 0;

/* Latest confirmed state from CM7 (ipc_cm7_to_cm4_t), refreshed each poll. */
static ipc_cm7_to_cm4_t cm7_status;

static void ipc_push(int ptt)
{
    ipc_cm4_to_cm7_t cmd;
    cmd.mode          = desired_mode;
    cmd.ptt           = ptt;
    cmd.cw_pitch_hz    = desired_cw_pitch_hz;
    cmd.cw_wpm         = desired_cw_wpm;
    cmd.cw_keyer_type  = desired_cw_keyer_type;
    cmd.mic_gain       = desired_mic_gain;
    ipc_cm4_to_cm7_write(&cmd);
}

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

/* Uncalibrated relative S-meter, estimated off cm7_status.rx_rms (0.0-1.0
   full-scale, post-demod - forwarded from CM7 via the IPC mailbox). Same
   placeholder -20 dBFS = S9, 6 dB/S-unit convention as F746; revisit once
   real RF AGC/S-meter hardware exists. */
static void format_smeter(char *buf, size_t bufsz)
{
    float level = cm7_status.rx_rms;
    if (level < 0.00003f) level = 0.00003f;
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
    snprintf(l_freq_sm, sizeof(l_freq_sm), "%s F:%sHZ", cm7_status.tx_active ? "TX" : "RX", freq_grp);
    snprintf(l_freq_big, sizeof(l_freq_big), "%s", freq_grp);

    char fn_val[16];
    switch (focus)
    {
    case FOCUS_FREQ:    snprintf(fn_val, sizeof(fn_val), "STEP %luHZ", (unsigned long)step_table[step_idx]); break;
    case FOCUS_VOLUME:  snprintf(fn_val, sizeof(fn_val), "VOL %d", volume); break;
    case FOCUS_MIC:     snprintf(fn_val, sizeof(fn_val), "MIC %d", (int)desired_mic_gain); break;
    case FOCUS_CWPITCH: snprintf(fn_val, sizeof(fn_val), "PITCH %dHZ", (int)desired_cw_pitch_hz); break;
    case FOCUS_KEYTYPE: snprintf(fn_val, sizeof(fn_val), "KEY %s", keyer_type_name[desired_cw_keyer_type]); break;
    case FOCUS_POWER:   snprintf(fn_val, sizeof(fn_val), "PWR %d", tx_power_pct); break;
    case FOCUS_RIT:     snprintf(fn_val, sizeof(fn_val), "RIT %ldHZ", (long)rit_offset_hz); break;
    default:            fn_val[0] = 0; break;
    }
    /* USB Audio routing isn't ported yet (see main.c's porting notes) -
       analog is the only real path right now, so this shows fixed rather
       than reading a live audio_source. */
    snprintf(l_audio_fn, sizeof(l_audio_fn), "A:ANLG FN:%s", fn_val);

    snprintf(l_mode, sizeof(l_mode), "M:%s %s", mode_name(cm7_status.mode), filt_name[filt_idx]);

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

/* console.c's "mode" command - the only other writer of desired_mode
   besides BTN_MODE in hmi_poll() above, so it goes through the same
   ipc_push() path rather than writing the IPC command directly, keeping
   one source of truth (otherwise the next unrelated hmi_poll()-driven
   push would silently overwrite a console-set mode with hmi.c's own
   stale idea of it). */
void hmi_set_mode(int32_t mode)
{
    desired_mode = mode;
    ipc_push(ptt_now_level);
}

void hmi_init(I2C_HandleTypeDef *oled_i2c)
{
    encoder_init();
    buttons_init();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = PTT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PTT_PORT, &gpio);

    oled_init(oled_i2c);
    ipc_push(0);   /* announce our defaults to CM7 before the first redraw */
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

    ipc_cm7_to_cm4_read(&cm7_status);

    int32_t enc_now   = encoder_poll();
    int32_t enc_delta = enc_now - enc_last;
    enc_last = enc_now;

    uint16_t btn_now  = buttons_read();
    uint16_t btn_edge = (uint16_t)(btn_now & ~btn_prev);
    btn_prev = btn_now;
    if (btn_edge && (now_tick - btn_debounce_tick) < 150)
        btn_edge = 0;
    if (btn_edge)
        btn_debounce_tick = now_tick;

    /* PTT: press-and-hold, not a toggle, so this stays level-based rather
       than edge-based like the numbered buttons. */
    uint8_t ptt_now = (HAL_GPIO_ReadPin(PTT_PORT, PTT_PIN) == GPIO_PIN_SET) ? 1 : 0;
    if (ptt_now != ptt_prev && (now_tick - ptt_debounce_tick) >= 30)
    {
        ptt_prev = ptt_now;
        ptt_debounce_tick = now_tick;
        ptt_now_level = ptt_now || tune_active;
        ipc_push(ptt_now_level);
        oled_last_tick = now_tick;
        redraw();
    }

    if (locked)
    {
        if (btn_edge & (1u << (BTN_LOCK - 1)))
        {
            locked = 0;
            redraw();
        }
        return;
    }

    int changed = 0;

    /* S-meter (and anything else continuously live) needs its own
       periodic redraw trigger - 200 ms (5 Hz) is enough for a meter. */
    {
        static uint32_t meter_tick = 0;
        if ((now_tick - meter_tick) >= 200)
        {
            meter_tick = now_tick;
            changed = 1;
        }
    }

    /* CM7's confirmed mode/tx_active can change from outside this poll
       loop's own edits too (a future CAT layer, once ported) - pick up any
       change the same way a local edit would, same reasoning as F746's
       console-driven-change handling. */
    {
        static int32_t last_seen_mode = -1;
        static int     last_seen_tx   = -1;
        if (cm7_status.mode != last_seen_mode) { last_seen_mode = cm7_status.mode; changed = 1; }
        if (cm7_status.tx_active != last_seen_tx) { last_seen_tx = cm7_status.tx_active; changed = 1; }
    }

    if (btn_edge & (1u << (BTN_FUNCTION - 1)))
    {
        focus = (focus == FOCUS_POWER) ? FOCUS_FREQ : (focus_t)(focus + 1);
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_MODE - 1)))
    {
        int next = cm7_status.mode + 1;
        if (next >= mode_count()) next = 0;
        desired_mode = next;
        ipc_push(ptt_now_level);
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
           mode's real TX chain produces. Combined with the physical PTT
           level via ipc_push()'s ptt argument (either one asking for TX
           is enough), same as F746 routed both through radio_tx_on/off(). */
        tune_active = !tune_active;
        ptt_now_level = ptt_prev || tune_active;
        ipc_push(ptt_now_level);
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_LOCK - 1)))
    {
        locked = 1;
        changed = 1;
    }
    if (btn_edge & (1u << (BTN_ENTER - 1)))
    {
        /* USB Audio routing toggle on F746 - not wired up here yet (see
           main.c's porting notes: this project's USB Audio class doesn't
           have F746's composite duplex glue yet). Left as a no-op rather
           than removed, so the button mapping stays stable once it is. */
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
            desired_mic_gain += (float)enc_delta;
            if (desired_mic_gain < 1.0f)   desired_mic_gain = 1.0f;
            if (desired_mic_gain > 200.0f) desired_mic_gain = 200.0f;
            ipc_push(ptt_now_level);
            break;
        case FOCUS_CWPITCH:
            desired_cw_pitch_hz += (float)(enc_delta * 10);
            if (desired_cw_pitch_hz < CW_PITCH_MIN_HZ) desired_cw_pitch_hz = CW_PITCH_MIN_HZ;
            if (desired_cw_pitch_hz > CW_PITCH_MAX_HZ) desired_cw_pitch_hz = CW_PITCH_MAX_HZ;
            ipc_push(ptt_now_level);
            break;
        case FOCUS_KEYTYPE:
        {
            /* A 3-item discrete menu, not an analog value - one step per
               poll interval regardless of how far the encoder turned. */
            int kt = desired_cw_keyer_type + ((enc_delta > 0) ? 1 : -1);
            if (kt < 0) kt = 2;
            if (kt > 2) kt = 0;
            desired_cw_keyer_type = kt;
            ipc_push(ptt_now_level);
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
