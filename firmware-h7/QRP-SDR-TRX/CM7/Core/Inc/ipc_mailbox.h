#ifndef IPC_MAILBOX_H
#define IPC_MAILBOX_H

#include <stdint.h>

/*
 * CM4 <-> CM7 control/telemetry mailbox.
 *
 * Needed because the HMI (mode/filter/CW settings via the encoder and
 * buttons) lives on CM4, but the actual mode state and DSP chains live on
 * CM7 - a plain function call across cores isn't possible, so control
 * changes and RX telemetry cross through a small shared-memory struct pair
 * in SRAM4 (D3 domain, 0x38000000-0x3800FFFF, reachable by both cores).
 *
 * Deliberately NOT OpenAMP/RPMsg: the traffic here is low-rate control and
 * status (a mode change on a button press, an S-meter update a few times a
 * second), not bulk data - a full messaging framework is more machinery
 * than this needs. Each direction has exactly one writer core and one
 * reader core, so a lock-free seqlock (odd sequence = write in progress,
 * even = stable; reader retries if the sequence changes mid-read) is
 * enough - no HSEM needed for the mailbox itself (HSEM is already used
 * elsewhere for the CubeMX-generated dual-core boot sync, HSEM_ID_0 - this
 * module doesn't touch that).
 *
 * Message set below covers what's already known to be needed from porting
 * modes.c/filters.c/tx_chain.c - deliberately not speculatively extended
 * beyond that; add fields as hmi.c's actual requirements become concrete
 * during its own port.
 */

/* CM4 -> CM7: control changes, written by the HMI (buttons.c/encoder.c/
   hmi.c) whenever the operator changes something. */
typedef struct {
    volatile uint32_t seq;         /* even = stable, odd = write in progress */
    int32_t  mode;                 /* MODE_* from modes.h */
    int32_t  ptt;                  /* 1 = TX requested (voice-mode PTT from
                                       the HMI - CW's own live key/paddle on
                                       CM7 bypasses this, same as F746 where
                                       cw_keyer_poll() calls radio_tx_on()
                                       directly rather than through PTT) */
    float    cw_pitch_hz;
    int32_t  cw_wpm;
    int32_t  cw_keyer_type;        /* 0=straight, 1=iambic A, 2=iambic B */
    float    mic_gain;             /* 1..200, see filters.h */
} ipc_cm4_to_cm7_t;

/* CM7 -> CM4: RX telemetry + state, written by main.c's audio processing
   loop (or a periodic tick within it - see ipc_mailbox.c) for the HMI's
   S-meter and RX/TX indicator. */
typedef struct {
    volatile uint32_t seq;
    float    rx_rms;               /* 0.0-1.0 full-scale, post-demod - what
                                       the HMI's S-meter reads */
    int32_t  tx_active;
    int32_t  mode;                 /* echoed back, so the HMI can confirm a
                                       mode-change request actually landed */
    int32_t  freedv_synced;
    float    freedv_snr_db;
    float    rx_load_pct;          /* process_block() worst-case time as a
                                       % of the real-time budget for the
                                       active block size/rate */
    float    rx_fdv_load_pct;      /* same, for just the FreeDV analysis
                                       path within process_block() */
    uint32_t rx_overruns;          /* main.c's ADC ring fell behind the DMA
                                       write pointer - lost modem samples,
                                       forces FreeDV to reacquire sync */
    uint32_t freedv_underruns;     /* freedv_chain.c's playback jitter
                                       buffer ran dry and had to rebuild its
                                       prefill cushion before speech audio
                                       resumes - the "audio only seldom
                                       comes through" symptom */
    float    poll_gap_us;          /* worst-case time between successive
                                       passes through main.c's buffered-mode
                                       dispatch check - diagnostic for
                                       whether the main loop is polling the
                                       ADC/DAC ring promptly or stalling */
    int32_t  freedv_nin_min;       /* min/max/avg of freedv_nin() since */
    int32_t  freedv_nin_max;       /* freedv_chain_init() - symmetric */
    int32_t  freedv_nin_avg;       /* jitter vs a real rate mismatch */
    uint32_t freedv_in_drops;      /* 2400B's in_fifo overflowed, samples
                                       silently dropped (not queued) */
    int32_t  clock_src_khz;        /* HSE input frequency actually in use,
                                       0 = fell back to the internal HSI */
    float    freedv_foff_hz;       /* DIAGNOSTIC 2026-09-02, 2400B sync
                                       investigation: estimated frequency
                                       offset - large/inconsistent values
                                       point at a mirrored/misplaced
                                       spectrum rather than a quality
                                       problem */
    float    freedv_sync_metric;   /* continuous 0-1 sync quality behind
                                       the binary freedv_synced flag */
    float    freedv_clock_ppm;     /* codec2's own tx/rx sample-clock
                                       offset estimate, ppm */
} ipc_cm7_to_cm4_t;

/*
 * Must be called once at startup on BOTH cores before any read/write
 * below. On CM7 this configures an MPU region marking SRAM4 as Device
 * memory (non-cacheable, non-bufferable) - without it, CM7's D-cache (once
 * enabled - not yet, as of this port; see the porting notes) could hold a
 * stale or torn view of what CM4 last wrote, since SRAM4 isn't covered by
 * the default cacheable memory map the way AXI SRAM is. CM4 has no data
 * cache, so this is a no-op there.
 */
void ipc_mailbox_init(void);

/* CM4 side: write a new command (call after any HMI-driven change).
   CM7 side: read the latest command (call periodically, e.g. once per
   main-loop pass - not audio-block rate, this is control-plane only). */
void ipc_cm4_to_cm7_write(const ipc_cm4_to_cm7_t *cmd);
void ipc_cm4_to_cm7_read(ipc_cm4_to_cm7_t *out);

/* CM7 side: write current telemetry (call at a UI-relevant rate, e.g.
   every 50-100 ms from the main loop - not every audio block; the HMI
   doesn't need video-rate updates any more than hmi.c's own redraw did on
   F746). CM4 side: read the latest telemetry. */
void ipc_cm7_to_cm4_write(const ipc_cm7_to_cm4_t *status);
void ipc_cm7_to_cm4_read(ipc_cm7_to_cm4_t *out);

#endif
