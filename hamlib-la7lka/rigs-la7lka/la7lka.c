/*
 *  Hamlib backend for the LA7LKA QRP SDR TRX (homebrew STM32 HF SDR
 *  transceiver, https://github.com/LA7LKA/SDR-TRX).
 *
 *  Its own standalone backend family (RIG_LA7LKA) rather than nested
 *  inside an existing one: this radio does not speak any real commercial
 *  rig's protocol and shares no code with kenwood.c or any other
 *  backend, so grouping it under e.g. Kenwood would be misleading in
 *  rigctl listings even though the family bucket is otherwise just a
 *  compiled-in loader mechanism.
 *
 *  The radio speaks a small, self-designed, Kenwood-*flavored* ASCII
 *  subset over a USB CDC-ACM virtual serial port (firmware side:
 *  cat_exec()/cat_poll() in Core/Src/main.c). It is deliberately NOT
 *  byte-compatible with any real Kenwood rig's spec - reusing generic
 *  kenwood_*() functions would require matching real Kenwood's 38-byte
 *  IF; status string exactly, which buys nothing here since both firmware
 *  and this backend are written by the same project and can agree on
 *  whatever's convenient. Framing differs from real Kenwood too: real
 *  Kenwood commands are bare ';'-terminated with no newline; this radio's
 *  CDC line reader only dispatches on '\r'/'\n', so every command sent
 *  here appends a trailing '\n' after the conventional ';'.
 *
 *  Wire protocol (must stay in sync with main.c's cat_exec()):
 *    MD;/MDx;  - get/set mode. Digits: 1=LSB 2=USB 3=CW 4=FM 5=AM.
 *    FA;/FB;   - get/set VFO frequency, 11-digit Hz. Cosmetic only - the
 *                radio has no real LO/VFO hardware yet, this just echoes
 *                a UI placeholder (see hmi.c's vfo_freq_hz).
 *    TX;/RX;   - PTT on/off, no reply either way.
 *    IF;       - compact custom status: "IF" + 11-digit freq + 1 mode
 *                digit + 1 PTT digit ('1'=TX/'0'=RX) + ";". NOT real
 *                Kenwood's IF; layout.
 *    ID;       - fixed "ID019;" reply.
 */

#include "hamlib/config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hamlib/rig.h"
#include "hamlib/port.h"
#include "hamlib/rig_state.h"
#include "iofunc.h"
#include "register.h"

/* Byte layout of the IF; reply payload, after stripping the trailing
   '\r'/'\n' and ';' (see la7lka_transaction()) - named offsets rather
   than magic numbers since this is exactly the kind of thing that's easy
   to get subtly wrong. Must stay in sync with main.c's cat_exec(). */
#define LA7LKA_IF_FREQ_OFFSET  2
#define LA7LKA_IF_FREQ_LEN     11
#define LA7LKA_IF_MODE_OFFSET  (LA7LKA_IF_FREQ_OFFSET + LA7LKA_IF_FREQ_LEN)   /* 13 */
#define LA7LKA_IF_PTT_OFFSET   (LA7LKA_IF_MODE_OFFSET + 1)                    /* 14 */
#define LA7LKA_IF_LEN          (LA7LKA_IF_PTT_OFFSET + 1)                     /* 15 */

static const struct { rmode_t mode; char kw; } la7lka_mode_table[] = {
    { RIG_MODE_LSB, '1' },
    { RIG_MODE_USB, '2' },
    { RIG_MODE_CW,  '3' },
    { RIG_MODE_FM,  '4' },
    { RIG_MODE_AM,  '5' },
};
#define LA7LKA_MODE_TABLE_LEN (sizeof(la7lka_mode_table) / sizeof(la7lka_mode_table[0]))

static char la7lka_mode_to_kw(rmode_t mode)
{
    size_t i;

    for (i = 0; i < LA7LKA_MODE_TABLE_LEN; i++)
    {
        if (la7lka_mode_table[i].mode == mode) return la7lka_mode_table[i].kw;
    }
    return 0;
}

static rmode_t la7lka_kw_to_mode(char c)
{
    size_t i;

    for (i = 0; i < LA7LKA_MODE_TABLE_LEN; i++)
    {
        if (la7lka_mode_table[i].kw == c) return la7lka_mode_table[i].mode;
    }
    return RIG_MODE_NONE;
}

/* One shared transaction helper for every callback below, modeled on
   netrigctl_transaction()'s I/O pattern (rig_flush -> write_block ->
   read_string), not on kenwood_transaction(). cmd is the bare command,
   e.g. "FA" or "FA00014200000" - ';' and '\n' are appended here. When
   expect_reply is 0 (every SET command - this radio never replies to
   those, matching real Kenwood's fire-and-forget convention), the read
   step is skipped entirely: blocking on read_string() for a reply that
   never arrives would just burn the full timeout on every single set_*()
   call and could surface as a spurious failure to callers. */
static int la7lka_transaction(RIG *rig, const char *cmd, char *reply, size_t replysz, int expect_reply)
{
    hamlib_port_t *rp = RIGPORT(rig);
    char buf[32];
    size_t len = strlen(cmd);
    int ret;

    if (len == 0 || len >= sizeof(buf) - 2) return -RIG_EINVAL;

    memcpy(buf, cmd, len);
    buf[len++] = ';';
    buf[len++] = '\n';

    rig_flush(rp);

    ret = write_block(rp, (const unsigned char *)buf, len);
    if (ret != RIG_OK) return ret;

    if (!expect_reply) return RIG_OK;

    ret = read_string(rp, (unsigned char *)reply, replysz, "\n", 1, 0, 1);
    if (ret < 0) return ret;

    while (ret > 0 && (reply[ret - 1] == '\n' || reply[ret - 1] == '\r')) ret--;
    if (ret > 0 && reply[ret - 1] == ';') ret--;
    reply[ret] = 0;

    return RIG_OK;
}

static int la7lka_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    char cmd[16];

    (void)vfo;
    snprintf(cmd, sizeof(cmd), "FA%011lu", (unsigned long)freq);
    return la7lka_transaction(rig, cmd, NULL, 0, 0);
}

static int la7lka_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    char reply[32];
    int ret;

    (void)vfo;
    ret = la7lka_transaction(rig, "FA", reply, sizeof(reply), 1);
    if (ret != RIG_OK) return ret;
    if (strlen(reply) < (size_t)(LA7LKA_IF_FREQ_OFFSET + LA7LKA_IF_FREQ_LEN)) return -RIG_EPROTO;

    *freq = (freq_t)strtoull(reply + LA7LKA_IF_FREQ_OFFSET, NULL, 10);
    return RIG_OK;
}

static int la7lka_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
    char cmd[8];
    char kw = la7lka_mode_to_kw(mode);

    (void)vfo;
    (void)width;
    if (!kw) return -RIG_EINVAL;

    snprintf(cmd, sizeof(cmd), "MD%c", kw);
    return la7lka_transaction(rig, cmd, NULL, 0, 0);
}

static int la7lka_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
    char reply[8];
    int ret;

    (void)vfo;
    ret = la7lka_transaction(rig, "MD", reply, sizeof(reply), 1);
    if (ret != RIG_OK) return ret;
    if (strlen(reply) < 3) return -RIG_EPROTO;

    *mode = la7lka_kw_to_mode(reply[2]);
    *width = kHz(2.4);   /* no real per-mode filter data to report yet */
    return RIG_OK;
}

static int la7lka_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
    (void)vfo;
    return la7lka_transaction(rig, ptt == RIG_PTT_ON ? "TX" : "RX", NULL, 0, 0);
}

static int la7lka_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt)
{
    char reply[24];
    int ret;

    (void)vfo;
    ret = la7lka_transaction(rig, "IF", reply, sizeof(reply), 1);
    if (ret != RIG_OK) return ret;
    if (strlen(reply) < (size_t)LA7LKA_IF_LEN) return -RIG_EPROTO;

    *ptt = (reply[LA7LKA_IF_PTT_OFFSET] == '1') ? RIG_PTT_ON : RIG_PTT_OFF;
    return RIG_OK;
}

static const char *la7lka_get_info(RIG *rig)
{
    static char reply[16];
    int ret;

    ret = la7lka_transaction(rig, "ID", reply, sizeof(reply), 1);
    if (ret != RIG_OK) return NULL;
    return reply;
}

struct rig_caps la7lka_caps =
{
    RIG_MODEL(RIG_MODEL_LA7LKA_TRX),
    .model_name       = "QRP SDR TRX",
    .mfg_name         = "LA7LKA",
    .version          = "0.1",
    .copyright        = "LGPL",
    .status           = RIG_STATUS_BETA,
    .rig_type         = RIG_TYPE_TRANSCEIVER,
    .ptt_type         = RIG_PTT_RIG,
    .dcd_type         = RIG_DCD_NONE,
    .port_type        = RIG_PORT_SERIAL,
    .serial_rate_min  = 9600,
    .serial_rate_max  = 115200,
    .serial_data_bits = 8,
    .serial_stop_bits = 1,
    .serial_parity    = RIG_PARITY_NONE,
    .serial_handshake = RIG_HANDSHAKE_NONE,
    .write_delay      = 0,
    .post_write_delay = 0,
    .timeout          = 500,
    .retry            = 3,
    .targetable_vfo   = RIG_TARGETABLE_NONE,   /* single VFO, no split/dual-VFO support yet */

    .rx_range_list1 = {
        {kHz(1800), MHz(30), RIG_MODE_CW | RIG_MODE_USB | RIG_MODE_LSB | RIG_MODE_AM | RIG_MODE_FM,
         -1, -1, RIG_VFO_A},
        RIG_FRNG_END,
    },
    .tx_range_list1 = {
        {kHz(1800), MHz(30), RIG_MODE_CW | RIG_MODE_USB | RIG_MODE_LSB | RIG_MODE_AM | RIG_MODE_FM,
         1000, 5000, RIG_VFO_A},   /* 1-5 W, matches this project's QRP power budget */
        RIG_FRNG_END,
    },
    .rx_range_list2 = { RIG_FRNG_END, },
    .tx_range_list2 = { RIG_FRNG_END, },

    .set_freq = la7lka_set_freq,
    .get_freq = la7lka_get_freq,
    .set_mode = la7lka_set_mode,
    .get_mode = la7lka_get_mode,
    .set_ptt  = la7lka_set_ptt,
    .get_ptt  = la7lka_get_ptt,
    .get_info = la7lka_get_info,

    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS,
};

DECLARE_INITRIG_BACKEND(la7lka)
{
    rig_register(&la7lka_caps);
    return RIG_OK;
}
