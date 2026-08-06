/**
  * USB Audio Class 1.0, bidirectional (mono, 16-bit, 48 kHz fixed).
  *
  * Hand-written, not a CubeMX/ST-template class: ST's stock USBD_AUDIO
  * (Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO) only implements
  * the speaker (host->device, OUT) direction. This project also needs the
  * microphone direction (device->host, IN) so RX audio can reach a PC
  * without an analog cable, so both directions are implemented here in one
  * custom class: one AudioControl interface collecting two AudioStreaming
  * interfaces (IF1 = OUT/speaker, IF2 = IN/microphone).
  *
  * 48 kHz was picked to match this project's existing ADC/DAC sample rate
  * exactly (TIM6 already paces both at 48 kHz) - one sample per USB
  * full-speed 1 ms frame is exactly 48 samples, so no fractional-packet
  * feedback endpoint is needed for the nominal rate. Clock drift between
  * the STM32's TIM6 and the USB host's clock is not corrected (no rate
  * feedback implemented) - fine for bench-test sessions, would need a
  * feedback/adaptive endpoint for long unattended runs.
  */
#ifndef __USBD_AUDIO_DUPLEX_H
#define __USBD_AUDIO_DUPLEX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"

#define USBD_AD_FREQ                    48000U
#define USBD_AD_BINTERVAL               0x01U

/* One sample (mono, 16-bit) per 1 kHz USB frame at 48 kHz = 96 bytes/ms,
   exactly, no jitter. */
#define USBD_AD_PACKET_SZE              ((USBD_AD_FREQ / 1000U) * 2U)

#define USBD_AD_OUT_EP                  0x01U   /* speaker: PC -> radio TX audio in  */
#define USBD_AD_IN_EP                   0x81U   /* mic:     radio RX audio -> PC     */

/* Note: multi-packet block arming (arm N packets per transfer so the CPU
   gets an N ms window instead of 1 ms) was tried together with
   dma_enable=1 and reverted - DataOut() then never fired at all on real
   hardware. See the comment in main.c's MX_USB_OTG_FS_PCD_Init(). One
   packet per transfer, slave mode, is what reliably passes audio here. */

/* Ring buffer depths, in bytes. This struct lives in USBD_static_malloc()'s
   fixed buffer (see usbd_conf.c), i.e. it's permanent .bss regardless of
   whether USB audio is even in use, and this project's FreeDV heap
   (OFDM+LDPC tables) runs close to the edge - a 72-packet (6912 B) OUT
   ring, sized for FreeDV's 1920-sample TX bursts, once ate back exactly
   the RAM headroom an earlier fix had recovered and reproduced a
   hang-on-FreeDV-mode-switch bug (confirmed on hardware: heap_left
   109664 = FreeDV fine, ~104256 = FreeDV hangs). Values below are chosen
   to stay well clear of that known-bad point:
     - 16 packets (1536 B) confirmed safe (heap_left 109664/247020 B bss)
     - 24 packets (2304 B) confirmed safe (heap_left recovered, 247812 B bss)
     - 72 packets (6912 B) confirmed hangs (~104256 B heap_left, 252420 B bss)
   32 packets (3072 B, +768 B over the 24-packet point) keeps roughly 4 KB
   of margin below the known-hang point while giving real dropouts (seen
   on a waterfall: correct 1 kHz tone, but chopped) more room to absorb
   ordinary delivery jitter from the host before the cushion empties.
   FreeDV's own 1920-sample TX bursts still won't fit even at 32 packets -
   that needs a mode-aware ring, not attempted here - so FreeDV TX over
   USB speaker audio remains a known limitation, not silently broken.
   Must stay an exact multiple of USBD_AD_PACKET_SZE: the ISR relies on
   the write pointer always landing on a packet boundary with a full
   packet's room left to the end of the buffer. */
#define USBD_AD_OUT_RING_SZE            (USBD_AD_PACKET_SZE * 32U)   /* ~32 ms */

/* IN ring: grown from 16 to 48 packets 2026-08-06 to stop
   USBD_AUDIO_DUPLEX_FeedMic() self-overwriting mid-write - it takes a
   whole RX block in one call with no bounds checking against the ring,
   and every buffered/FreeDV RX mode (1600/2400B/700D/700E, all
   BLOCK_SIZE 3840) feeds 1920 samples/3840 bytes per call, more than
   double the 16-packet ring's 1536-byte capacity - confirmed on hardware
   as "just garble"/chopped audio on FreeDV1600 (SSB's much smaller
   256-sample calls never hit it, which is why this went unnoticed for so
   long). That corruption bug is fixed separately in usbd_audio_duplex.c
   (FeedMic() now caps to actual free space instead of wrapping into
   unset data) and is independent of ring size - reverted back to 16
   packets same day after 48 (4608 B, +3072 B) reproduced a hard
   fault/MemManage fault starting FreeDV1600 decode, in *both* src usb
   and src analog - so it wasn't specific to the USB code path, it was
   heap/stack pressure the extra 3 KB tipped over, hitting some
   memory-safety gap in codec2's 1600 path (a different specific bug
   from 700D's LDPC fragmentation, since 1600 never touches that decoder
   at all).
   Getting FreeDV RX-over-USB genuinely clean, not just non-corrupted,
   needs that gap found first - a ring-size change alone isn't enough. */
#define USBD_AD_IN_RING_SZE             (USBD_AD_PACKET_SZE * 16U)   /* ~16 ms */

#define USBD_AD_CONFIG_DESC_SIZ         0xC0U   /* 192 bytes, see .c for the byte-by-byte tally */

typedef enum
{
  USBD_AD_OFFSET_NONE = 0,
  USBD_AD_OFFSET_HALF,
  USBD_AD_OFFSET_FULL,
  USBD_AD_OFFSET_UNKNOWN,
} USBD_AD_OffsetTypeDef;

typedef struct
{
  uint8_t cmd;
  uint8_t data[USB_MAX_EP0_SIZE];
  uint8_t len;
  uint8_t unit;
} USBD_AD_ControlTypeDef;

typedef struct
{
  uint32_t out_alt_setting;
  uint32_t in_alt_setting;

  /* OUT (speaker) path: host pushes data, ring tracked like ST's stock
     speaker class (wr_ptr advances on DataOut, rd_enable/offset gate when
     the application starts consuming). 4-byte alignment is a hard OTG
     core DMA requirement (dma_enable=1, see main.c) - the natural struct
     layout already gives this (two uint32_t fields precede it), but it's
     pinned explicitly rather than left as an accident of field order. */
  uint8_t  out_buf[USBD_AD_OUT_RING_SZE] __attribute__((aligned(4)));
  uint16_t out_wr_ptr;
  uint16_t out_rd_ptr;
  uint8_t  out_rd_enable;
  USBD_AD_OffsetTypeDef out_offset;

  /* IN (mic) path: application writes fresh RX audio at in_wr_ptr; the
     class pulls USBD_AD_PACKET_SZE bytes from in_rd_ptr every time the
     previous IN transfer completes (chained from DataIn, first armed by
     SET_INTERFACE alt=1 in Setup - see usbd_audio_duplex.c). Same DMA
     alignment requirement as out_buf above. */
  uint8_t  in_buf[USBD_AD_IN_RING_SZE] __attribute__((aligned(4)));
  uint16_t in_wr_ptr;
  uint16_t in_rd_ptr;

  USBD_AD_ControlTypeDef control;
} USBD_AUDIO_DUPLEX_HandleTypeDef;

typedef struct
{
  int8_t (*Init)(uint32_t AudioFreq);
  int8_t (*DeInit)(void);
  int8_t (*MuteCtl)(uint8_t cmd);
} USBD_AUDIO_DUPLEX_ItfTypeDef;

extern USBD_ClassTypeDef USBD_AUDIO_DUPLEX;
#define USBD_AUDIO_DUPLEX_CLASS &USBD_AUDIO_DUPLEX

uint8_t USBD_AUDIO_DUPLEX_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_AUDIO_DUPLEX_ItfTypeDef *fops);

/* Both are ring-buffer pump/pull calls the application drives from its own
   ~10.6 ms (512-sample) DSP block cadence, not from USB ISR context - they
   only touch plain ring indices, so calling them from the main loop is
   safe as long as each index is only ever written from one side (app
   writes in_wr_ptr/reads out_rd_ptr are not touched here; the USB ISR side
   owns in_rd_ptr/out_wr_ptr). */

/* App -> mic (IN, radio RX audio -> PC). pcm16 mono 16-bit signed. */
void USBD_AUDIO_DUPLEX_FeedMic(USBD_HandleTypeDef *pdev, const int16_t *pcm16, uint32_t n_samples);

/* Speaker (OUT, PC -> radio TX audio) -> app. Fills pcm16 with n_samples
   samples, zero-padding (silence) if fewer are available. Returns the
   number of samples that were genuinely available (for underrun/level
   diagnostics); the output buffer is always fully written. */
uint32_t USBD_AUDIO_DUPLEX_GetSpeakerAudio(USBD_HandleTypeDef *pdev, int16_t *pcm16, uint32_t n_samples);

/* Drop any audio that piled up while nothing was consuming and start a
   fresh cushion - call when transmit begins. */
void USBD_AUDIO_DUPLEX_ResetSpeaker(USBD_HandleTypeDef *pdev);

/* Mic-direction counterpart: drop whatever's queued for the IN endpoint
   and have it start sending fresh samples from "now" - call whenever
   something on the app side could otherwise leave it out of step, e.g. a
   mode or audio-source change. */
void USBD_AUDIO_DUPLEX_ResetMic(USBD_HandleTypeDef *pdev);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_AUDIO_DUPLEX_H */
