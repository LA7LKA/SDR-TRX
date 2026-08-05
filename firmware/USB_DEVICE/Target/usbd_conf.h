/**
  * USB Device low-level configuration.
  * Hand-written glue layer for a single custom class (USBD_AUDIO_DUPLEX) on
  * USB_OTG_FS, since ST's stock USBD_AUDIO class is speaker-only (OUT) and
  * this project needs a bidirectional (speaker + mic) Audio Class device.
  */
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define USBD_MAX_NUM_INTERFACES              3U
#define USBD_MAX_NUM_CONFIGURATION           1U
/* Longest string we send is "QRP SDR TRX Audio" (18 chars -> 38-byte
   UTF-16 descriptor); 128 is already generous headroom, not the
   originally-copied 512 - same reasoning as the ring buffer trim in
   usbd_audio_duplex.h, this is permanent .bss competing with FreeDV's
   heap. */
#define USBD_MAX_STR_DESC_SIZ                128U
#define USBD_SELF_POWERED                    1U
#define USBD_DEBUG_LEVEL                     0U
#define USBD_LPM_ENABLED                     0U
#define USBD_SUPPORT_USER_STRING_DESC        0U
#define USBD_CLASS_USER_STRING_DESC          0U
#define USBD_MAX_CLASS_ENDPOINTS             5U
#define USBD_MAX_CLASS_INTERFACES            5U

/* Only one custom class is ever registered (non-composite path). */
#define USBD_MAX_SUPPORTED_CLASS             1U

/* USBD_Init()'s `id` parameter is stored but otherwise unused by this core
   version - kept for the conventional CubeMX-style call site. */
#define DEVICE_FS                            0U

/* Memory management macros - static allocation only. */
#define USBD_malloc         (void *)USBD_static_malloc
#define USBD_free           USBD_static_free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)   do { printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_UsrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)   do { printf("USBD ERROR: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_ErrLog(...) do {} while (0)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)   do { printf("USBD DEBUG: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_DbgLog(...) do {} while (0)
#endif

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
