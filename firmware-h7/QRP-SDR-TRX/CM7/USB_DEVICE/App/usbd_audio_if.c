/**
  * Application-side callbacks for the USBD_AUDIO_DUPLEX class. There's no
  * real BSP audio codec involved here (the "hardware" is main.c's own
  * ADC/DAC audio pipeline, fed/drained via USBD_AUDIO_DUPLEX_FeedMic() /
  * USBD_AUDIO_DUPLEX_GetSpeakerAudio() directly) - these three callbacks
  * are just what the class insists on calling, kept as no-ops.
  */
#include "usbd_audio_if.h"

static int8_t AudioIf_Init(uint32_t AudioFreq)
{
  (void)AudioFreq;
  return 0;
}

static int8_t AudioIf_DeInit(void)
{
  return 0;
}

static int8_t AudioIf_MuteCtl(uint8_t cmd)
{
  (void)cmd;
  return 0;
}

USBD_AUDIO_DUPLEX_ItfTypeDef USBD_AUDIO_IF_fops =
{
  AudioIf_Init,
  AudioIf_DeInit,
  AudioIf_MuteCtl,
};
