#include "hmi.h"
#include "oled.h"
#include "encoder.h"
#include "buttons.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

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
int  mode_count(void);
const char *mode_name(int mode);
void radio_set_mode(int mode);
int  radio_tx_on(void);
void radio_tx_off(void);
void cw_set_pitch(float hz);

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

typedef enum { FOCUS_FREQ, FOCUS_VOLUME, FOCUS_MIC, FOCUS_CWPITCH, FOCUS_POWER, FOCUS_RIT } focus_t;

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

static void redraw(void)
{
    char l0[24], l1[24], l2[24], l3[32];

    snprintf(l0, sizeof(l0), "%s %s", mode_name(MODE), filt_name[filt_idx]);
    snprintf(l1, sizeof(l1), "%sM F:%lu", band_name[band_idx], (unsigned long)vfo_freq_hz);

    switch (focus)
    {
    case FOCUS_FREQ:    snprintf(l2, sizeof(l2), "STEP:%lu", (unsigned long)step_table[step_idx]); break;
    case FOCUS_VOLUME:  snprintf(l2, sizeof(l2), "VOL:%d", volume); break;
    case FOCUS_MIC:     snprintf(l2, sizeof(l2), "MIC:%d", (int)mic_gain); break;
    case FOCUS_CWPITCH: snprintf(l2, sizeof(l2), "PITCH:%d", (int)cw_pitch_hz); break;
    case FOCUS_POWER:   snprintf(l2, sizeof(l2), "PWR:%d", tx_power_pct); break;
    case FOCUS_RIT:     snprintf(l2, sizeof(l2), "RIT:%ld", (long)rit_offset_hz); break;
    default:            l2[0] = 0; break;
    }

    l3[0] = 0;
    if (locked)       strcat(l3, "LOCK ");
    if (tx_active)    strcat(l3, "TX ");
    if (rit_active)   strcat(l3, "RIT ");
    if (split_active) strcat(l3, "SPLIT ");
    if (tune_active)  strcat(l3, "TUNE ");
    if (!l3[0])       strcpy(l3, "RX");

    oled_clear();
    oled_draw_text(0, 0, l0);
    oled_draw_text(0, 2, l1);
    oled_draw_text(0, 4, l2);
    oled_draw_text(0, 6, l3);
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
    /* BTN_ENTER: reserved for a menu system that doesn't exist yet. */

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
