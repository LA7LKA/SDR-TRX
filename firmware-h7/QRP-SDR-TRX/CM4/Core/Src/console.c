#include "console.h"
#include "hmi.h"
#include "modes.h"
#include "ipc_mailbox.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static UART_HandleTypeDef *console_uart;

#define LINE_MAX 63
static char             line_buf[LINE_MAX + 1];
static volatile uint8_t line_len;
static volatile uint8_t line_ready;
static volatile uint8_t rx_byte;

/* Bytes queued here by the RX ISR (typed-character/backspace echo) and
   drained by console_poll() in the main loop. The ISR must never call a
   blocking HAL_UART_Transmit() itself: if it preempts a transmit already
   in progress from the main loop (e.g. a command's response) and starts
   its own on the same handle, the two clobber each other's HAL state
   (gState, TxXferCount, pTxBuffPtr) - seen on hardware as garbage output
   whenever a keystroke happened to land mid-transmission. */
#define ECHO_BUF_SIZE 32
static char             echo_buf[ECHO_BUF_SIZE];
static volatile uint8_t echo_head;   /* ISR writes here */
static volatile uint8_t echo_tail;   /* console_poll() reads here */

static void echo_queue(const char *s, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t next = (uint8_t)((echo_head + 1) % ECHO_BUF_SIZE);
        if (next == echo_tail)
            break;   /* full - drop rather than block in ISR */
        echo_buf[echo_head] = s[i];
        echo_head = next;
    }
}

static void console_print(const char *s)
{
    HAL_UART_Transmit(console_uart, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void prompt(void)
{
    console_print("\r\n> ");
}

/* Short keywords rather than mode_name()'s display strings ("FreeDV 1600"
   etc.) - easier to type, and mode_name()'s spaces would need quoting. */
static const struct { const char *kw; int32_t mode; } mode_kw[] = {
    { "nbfm",         MODE_NBFM },
    { "fm",           MODE_NBFM },
    { "usb",          MODE_USB },
    { "lsb",          MODE_LSB },
    { "freedv1600",   MODE_FREEDV },
    { "freedv2400b",  MODE_FREEDV_2400B },
    { "freedv700d",   MODE_FREEDV_700D },
    { "freedv700e",   MODE_FREEDV_700E },
    { "am",           MODE_AM },
    { "cw",           MODE_CW },
};
#define MODE_KW_COUNT ((int)(sizeof(mode_kw) / sizeof(mode_kw[0])))

static void cmd_help(void)
{
    console_print("\r\ncommands:\r\n"
                  "  help             this text\r\n"
                  "  status           mode, RX level, FreeDV sync/SNR, CPU load\r\n"
                  "  mode <name>      nbfm usb lsb am cw freedv1600 freedv2400b freedv700d freedv700e\r\n");
}

static void cmd_status(void)
{
    ipc_cm7_to_cm4_t st;
    ipc_cm7_to_cm4_read(&st);

    char line[96];
    snprintf(line, sizeof(line), "\r\nmode=%s tx=%s rx_rms=%.4f\r\n",
             mode_name(st.mode), st.tx_active ? "yes" : "no", (double)st.rx_rms);
    console_print(line);

    if (st.freedv_synced || st.freedv_snr_db != 0.0f)
    {
        snprintf(line, sizeof(line), "freedv: sync=%s snr=%.1fdB metric=%.2f\r\n",
                 st.freedv_synced ? "yes" : "no", (double)st.freedv_snr_db,
                 (double)st.freedv_sync_metric);
        console_print(line);

        snprintf(line, sizeof(line), "freedv: foff=%.1fHz clock_offset=%.1fppm\r\n",
                 (double)st.freedv_foff_hz, (double)st.freedv_clock_ppm);
        console_print(line);
    }

    snprintf(line, sizeof(line), "cpu load: process_block=%.1f%% freedv_path=%.1f%%\r\n",
             (double)st.rx_load_pct, (double)st.rx_fdv_load_pct);
    console_print(line);

    snprintf(line, sizeof(line), "rx_overruns=%lu freedv_underruns=%lu\r\n",
             (unsigned long)st.rx_overruns, (unsigned long)st.freedv_underruns);
    console_print(line);

    if (st.clock_src_khz)
        snprintf(line, sizeof(line), "clock=HSE %ld kHz (%s)\r\n",
                 (long)st.clock_src_khz,
                 st.clock_src_khz == 25000 ? "X2 crystal, 20ppm"
                                           : "ST-LINK MCO, via osc amp");
    else
        snprintf(line, sizeof(line), "clock=HSI (internal RC, ~3300ppm off)\r\n");
    console_print(line);

    snprintf(line, sizeof(line), "poll_gap_max=%.1fus\r\n", (double)st.poll_gap_us);
    console_print(line);

    snprintf(line, sizeof(line), "freedv_nin: min=%ld max=%ld avg=%ld\r\n",
             (long)st.freedv_nin_min, (long)st.freedv_nin_max, (long)st.freedv_nin_avg);
    console_print(line);

    snprintf(line, sizeof(line), "freedv_in_drops=%lu\r\n", (unsigned long)st.freedv_in_drops);
    console_print(line);
}

static void cmd_mode(char *arg)
{
    if (!arg || !*arg)
    {
        console_print("\r\nusage: mode <name> - see 'help'\r\n");
        return;
    }
    for (char *p = arg; *p; p++) *p = (char)tolower((unsigned char)*p);

    for (int i = 0; i < MODE_KW_COUNT; i++)
    {
        if (strcmp(arg, mode_kw[i].kw) == 0)
        {
            hmi_set_mode(mode_kw[i].mode);
            char line[48];
            snprintf(line, sizeof(line), "\r\nmode -> %s\r\n", mode_name(mode_kw[i].mode));
            console_print(line);
            return;
        }
    }
    console_print("\r\nunknown mode - see 'help'\r\n");
}

static void dispatch(char *line)
{
    while (*line == ' ') line++;
    if (!*line) { prompt(); return; }

    char *arg = strchr(line, ' ');
    if (arg) { *arg = 0; arg++; while (*arg == ' ') arg++; }

    if      (strcmp(line, "help")   == 0) cmd_help();
    else if (strcmp(line, "status") == 0) cmd_status();
    else if (strcmp(line, "mode")   == 0) cmd_mode(arg);
    else
    {
        console_print("\r\nunknown command - try 'help'\r\n");
    }
    prompt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != console_uart)
        return;

    uint8_t c = rx_byte;

    if (c == '\r' || c == '\n')
    {
        if (line_len > 0 || c == '\r')
        {
            line_buf[line_len] = 0;
            line_ready = 1;
        }
    }
    else if (c == 0x08 || c == 0x7F)   /* backspace/DEL */
    {
        if (line_len > 0)
        {
            line_len--;
            echo_queue("\b \b", 3);
        }
    }
    else if (c >= 0x20 && c < 0x7F && line_len < LINE_MAX)
    {
        line_buf[line_len++] = (char)c;
        echo_queue((const char *)&c, 1);
    }

    HAL_UART_Receive_IT(console_uart, (uint8_t *)&rx_byte, 1);
}

/* Without this, any transient RX error (framing/noise/overrun - plausible
   on this bench wiring) leaves the receiver permanently dead: HAL's ISR
   calls this instead of HAL_UART_RxCpltCallback() on error, and if
   nothing re-arms HAL_UART_Receive_IT() here, the single outstanding
   receive is simply gone - every keystroke after that first error is
   silently dropped, for the rest of the boot. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != console_uart)
        return;

    __HAL_UART_CLEAR_PEFLAG(huart);
    HAL_UART_Receive_IT(console_uart, (uint8_t *)&rx_byte, 1);
}

void console_init(UART_HandleTypeDef *huart)
{
    console_uart = huart;
    line_len = 0;
    line_ready = 0;

    /* Print before arming the RX interrupt, not after - enabling
       HAL_UART_Receive_IT() while a blocking HAL_UART_Transmit() runs
       right after it corrupted the banner on hardware (banner truncated
       to a single garbage byte), so send it on a still-quiet UART. */
    console_print("\r\nqrp-sdr-trx console - type 'help'\r\n> ");

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_IT(console_uart, (uint8_t *)&rx_byte, 1);
}

void console_poll(void)
{
    while (echo_tail != echo_head)
    {
        uint8_t c = (uint8_t)echo_buf[echo_tail];
        HAL_UART_Transmit(console_uart, &c, 1, 10);
        echo_tail = (uint8_t)((echo_tail + 1) % ECHO_BUF_SIZE);
    }

    if (!line_ready)
        return;

    dispatch(line_buf);
    line_len = 0;
    line_ready = 0;
}
