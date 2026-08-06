/**
  * USB Audio Class 1.0, bidirectional. See usbd_audio_duplex.h for the
  * rationale (ST's stock USBD_AUDIO is speaker/OUT-only).
  *
  * Descriptor layout: one AudioControl interface (IF0) collecting two
  * AudioStreaming interfaces - IF1 = OUT ("speaker" terminal, PC sends TX
  * audio to the radio), IF2 = IN ("microphone" terminal, radio sends RX
  * audio to the PC). Mono, 16-bit, fixed 48 kHz (Format Type I,
  * bSamFreqType=1, single rate - no SET/GET sample-rate negotiation).
  */
#include "usbd_audio_duplex.h"
#include "usbd_ctlreq.h"

/* Speaker(OUT)-path diagnostics; defined further down, declared here
   because the USB ISR paths above their definition also update them. */
extern volatile uint32_t usb_spk_calls;
extern volatile uint32_t usb_spk_underruns;
extern volatile uint32_t usb_spk_avail;
extern volatile uint32_t usb_spk_avail_min;
extern volatile uint32_t usb_spk_rx_pkts;
extern volatile uint32_t usb_spk_nonzero_pkts;
extern volatile uint32_t usb_spk_laps;
extern volatile uint32_t usb_mic_underruns;
extern volatile uint32_t usb_mic_fade_ins;
extern volatile uint32_t usb_mic_fade_outs;

#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq) >> 8), (uint8_t)((frq) >> 16)

#define AUDIO_PACKET_SZE_LE(frq) \
  (uint8_t)((((frq) / 1000U) * 2U) & 0xFFU), (uint8_t)(((((frq) / 1000U) * 2U) >> 8) & 0xFFU)

static uint8_t USBD_AD_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AD_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AD_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_AD_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AD_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_AD_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AD_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AD_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AD_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AD_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AD_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AD_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AD_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AD_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

USBD_ClassTypeDef USBD_AUDIO_DUPLEX =
{
  USBD_AD_Init,
  USBD_AD_DeInit,
  USBD_AD_Setup,
  USBD_AD_EP0_TxReady,
  USBD_AD_EP0_RxReady,
  USBD_AD_DataIn,
  USBD_AD_DataOut,
  USBD_AD_SOF,
  USBD_AD_IsoINIncomplete,
  USBD_AD_IsoOutIncomplete,
  USBD_AD_GetCfgDesc,
  USBD_AD_GetCfgDesc,
  USBD_AD_GetCfgDesc,
  USBD_AD_GetDeviceQualifierDesc,
};

__ALIGN_BEGIN static uint8_t USBD_AD_CfgDesc[USBD_AD_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* ---- Configuration descriptor (9) ---- */
  0x09, USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(USBD_AD_CONFIG_DESC_SIZ), HIBYTE(USBD_AD_CONFIG_DESC_SIZ),
  0x03,                                  /* bNumInterfaces: AC + AS-out + AS-in */
  0x01,                                  /* bConfigurationValue */
  0x00,                                  /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,

  /* ---- IF0: standard AudioControl interface (9) ---- */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x00, 0x00, 0x00,                      /* bInterfaceNumber, bAlternateSetting, bNumEndpoints */
  0x01, 0x01, 0x00, 0x00,                /* Class=AUDIO, SubClass=AUDIOCONTROL, Protocol, iInterface */

  /* ---- Class-specific AC interface header (10: bInCollection=2) ---- */
  0x0A, 0x24, 0x01,                      /* bLength, CS_INTERFACE, HEADER */
  0x00, 0x01,                            /* bcdADC 1.00 */
  0x46, 0x00,                            /* wTotalLength = 70 (AC cluster only) */
  0x02,                                  /* bInCollection */
  0x01, 0x02,                            /* baInterfaceNr(1)=IF1, baInterfaceNr(2)=IF2 */

  /* ---- Input Terminal 1: USB streaming in (speaker path source) (12) ---- */
  0x0C, 0x24, 0x02,                      /* bLength, CS_INTERFACE, INPUT_TERMINAL */
  0x01,                                  /* bTerminalID */
  0x01, 0x01,                            /* wTerminalType 0x0101 USB streaming */
  0x00,                                  /* bAssocTerminal */
  0x01,                                  /* bNrChannels: mono */
  0x00, 0x00,                            /* wChannelConfig */
  0x00, 0x00,                            /* iChannelNames, iTerminal */

  /* ---- Feature Unit 2 (source=1), mute only (9) ---- */
  0x09, 0x24, 0x06,                      /* bLength, CS_INTERFACE, FEATURE_UNIT */
  0x02,                                  /* bUnitID */
  0x01,                                  /* bSourceID */
  0x01,                                  /* bControlSize */
  0x01, 0x00,                            /* bmaControls(master)=MUTE, bmaControls(ch1)=0 */
  0x00,                                  /* iTerminal */

  /* ---- Output Terminal 3 (source=2): physical speaker (9) ---- */
  0x09, 0x24, 0x03,                      /* bLength, CS_INTERFACE, OUTPUT_TERMINAL */
  0x03,                                  /* bTerminalID */
  0x01, 0x03,                            /* wTerminalType 0x0301 Speaker */
  0x00,                                  /* bAssocTerminal */
  0x02,                                  /* bSourceID */
  0x00,                                  /* iTerminal */

  /* ---- Input Terminal 4: physical microphone (mic path source) (12) ---- */
  0x0C, 0x24, 0x02,
  0x04,                                  /* bTerminalID */
  0x01, 0x02,                            /* wTerminalType 0x0201 Microphone */
  0x00,
  0x01,                                  /* bNrChannels: mono */
  0x00, 0x00,
  0x00, 0x00,

  /* ---- Feature Unit 5 (source=4), mute only (9) ---- */
  0x09, 0x24, 0x06,
  0x05,
  0x04,
  0x01,
  0x01, 0x00,
  0x00,

  /* ---- Output Terminal 6 (source=5): USB streaming out (mic path sink) (9) ---- */
  0x09, 0x24, 0x03,
  0x06,
  0x01, 0x01,                            /* wTerminalType 0x0101 USB streaming */
  0x00,
  0x05,
  0x00,

  /* ================= IF1: AudioStreaming OUT (speaker) ================= */

  /* Alt 0: zero-bandwidth (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x01, 0x00, 0x00,
  0x01, 0x02, 0x00, 0x00,                /* Class=AUDIO, SubClass=AUDIOSTREAMING */

  /* Alt 1: operational, 1 endpoint (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x01, 0x01, 0x01,
  0x01, 0x02, 0x00, 0x00,

  /* Class-specific AS General (7) */
  0x07, 0x24, 0x01,                      /* bLength, CS_INTERFACE, AS_GENERAL */
  0x01,                                  /* bTerminalLink = IT1 */
  0x01,                                  /* bDelay */
  0x01, 0x00,                            /* wFormatTag = PCM */

  /* Format Type I (11) */
  0x0B, 0x24, 0x02,                      /* bLength, CS_INTERFACE, FORMAT_TYPE */
  0x01,                                  /* bFormatType I */
  0x01,                                  /* bNrChannels: mono */
  0x02,                                  /* bSubFrameSize: 2 bytes */
  0x10,                                  /* bBitResolution: 16 */
  0x01,                                  /* bSamFreqType: 1 discrete rate */
  AUDIO_SAMPLE_FREQ(USBD_AD_FREQ),

  /* Standard AS ISO OUT endpoint (9) */
  0x09, USB_DESC_TYPE_ENDPOINT,
  USBD_AD_OUT_EP,
  0x0D,                                  /* bmAttributes: Isochronous, Synchronous, Data EP (fixed-size every SOF, no rate feedback) */
  AUDIO_PACKET_SZE_LE(USBD_AD_FREQ),
  USBD_AD_BINTERVAL,
  0x00, 0x00,

  /* Class-specific AS ISO OUT endpoint (7) */
  0x07, 0x25, 0x01,                      /* bLength, CS_ENDPOINT, EP_GENERAL */
  0x00, 0x00, 0x00, 0x00,

  /* ================= IF2: AudioStreaming IN (microphone) ================= */

  /* Alt 0: zero-bandwidth (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x02, 0x00, 0x00,
  0x01, 0x02, 0x00, 0x00,

  /* Alt 1: operational, 1 endpoint (9) */
  0x09, USB_DESC_TYPE_INTERFACE,
  0x02, 0x01, 0x01,
  0x01, 0x02, 0x00, 0x00,

  /* Class-specific AS General (7) */
  0x07, 0x24, 0x01,
  0x06,                                  /* bTerminalLink = OT6 */
  0x01,
  0x01, 0x00,

  /* Format Type I (11) */
  0x0B, 0x24, 0x02,
  0x01,
  0x01,
  0x02,
  0x10,
  0x01,
  AUDIO_SAMPLE_FREQ(USBD_AD_FREQ),

  /* Standard AS ISO IN endpoint (9) */
  0x09, USB_DESC_TYPE_ENDPOINT,
  USBD_AD_IN_EP,
  0x0D,
  AUDIO_PACKET_SZE_LE(USBD_AD_FREQ),
  USBD_AD_BINTERVAL,
  0x00, 0x00,

  /* Class-specific AS ISO IN endpoint (7) */
  0x07, 0x25, 0x01,
  0x00, 0x00, 0x00, 0x00,
};

__ALIGN_BEGIN static uint8_t USBD_AD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02, 0x00, 0x00, 0x00, USB_MAX_EP0_SIZE, 0x01, 0x00,
};

static uint8_t *USBD_AD_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AD_CfgDesc);
  return USBD_AD_CfgDesc;
}

static uint8_t *USBD_AD_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AD_DeviceQualifierDesc);
  return USBD_AD_DeviceQualifierDesc;
}

static uint8_t USBD_AD_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_DUPLEX_HandleTypeDef));
  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }
  USBD_memset(haudio, 0, sizeof(USBD_AUDIO_DUPLEX_HandleTypeDef));

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

  (void)USBD_LL_OpenEP(pdev, USBD_AD_OUT_EP, USBD_EP_TYPE_ISOC, USBD_AD_PACKET_SZE);
  pdev->ep_out[USBD_AD_OUT_EP & 0xFU].is_used = 1U;
  pdev->ep_out[USBD_AD_OUT_EP & 0xFU].bInterval = USBD_AD_BINTERVAL;

  (void)USBD_LL_OpenEP(pdev, USBD_AD_IN_EP, USBD_EP_TYPE_ISOC, USBD_AD_PACKET_SZE);
  pdev->ep_in[USBD_AD_IN_EP & 0xFU].is_used = 1U;
  pdev->ep_in[USBD_AD_IN_EP & 0xFU].bInterval = USBD_AD_BINTERVAL;

  haudio->out_offset = USBD_AD_OFFSET_UNKNOWN;

  if (((USBD_AUDIO_DUPLEX_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init(USBD_AD_FREQ) != 0)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Arm the OUT endpoint for the first speaker packet. (DMA + multi-packet
     block arming was tried and reverted here - see git history/main.c's
     MX_USB_OTG_FS_PCD_Init() - single-packet slave mode is what's proven
     to reliably pass audio.) */
  (void)USBD_LL_PrepareReceive(pdev, USBD_AD_OUT_EP, haudio->out_buf, USBD_AD_PACKET_SZE);

  /* The IN (mic) endpoint is deliberately NOT primed here. USBD_Init()
     runs at USB configuration time, typically seconds before any host
     driver actually opens the capture stream - an isochronous IN
     transfer armed while the host isn't yet polling that endpoint (still
     at AudioStreaming alt-setting 0) just goes stale: no host IN token
     ever claims it, so it never completes, so DataIn() never fires to
     chain the next packet, and the class silently never streams
     anything. The real, host-driven "the mic interface is now actually
     wanted" signal is SET_INTERFACE selecting alt 1, which is where the
     first transmit belongs - see USBD_AD_Setup(). */

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_AD_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  (void)USBD_LL_CloseEP(pdev, USBD_AD_OUT_EP);
  pdev->ep_out[USBD_AD_OUT_EP & 0xFU].is_used = 0U;
  (void)USBD_LL_CloseEP(pdev, USBD_AD_IN_EP);
  pdev->ep_in[USBD_AD_IN_EP & 0xFU].is_used = 0U;

  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_AUDIO_DUPLEX_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_AD_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t status_info = 0U;

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case 0x81U: /* GET_CUR */
          AD_REQ_GetCurrent(pdev, req);
          break;
        case 0x01U: /* SET_CUR */
          AD_REQ_SetCurrent(pdev, req);
          break;
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            uint8_t cur = (LOBYTE(req->wIndex) == 1U) ? (uint8_t)haudio->out_alt_setting
                                                        : (uint8_t)haudio->in_alt_setting;
            (void)USBD_CtlSendData(pdev, &cur, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (LOBYTE(req->wIndex) == 1U)
            {
              haudio->out_alt_setting = req->wValue;
              if (req->wValue == 1U)
              {
                /* Same bug, same fix, as the mic side below: the OUT
                   endpoint was only ever armed once, in USBD_Init(), at
                   USB configuration time. But PipeWire (and any other UAC1
                   host) leaves this interface at alt 0 and doesn't switch
                   it to alt 1 - doesn't actually start sending - until a
                   playback stream attaches, which can be seconds or
                   minutes later. By then there was no outstanding receive
                   left for DataOut() to ever fire from, so not a single
                   OUT packet was ever accepted, no matter what played to
                   the sink. Arm it here, when the host is actually about
                   to start sending. */
                (void)USBD_LL_PrepareReceive(pdev, USBD_AD_OUT_EP,
                                             &haudio->out_buf[haudio->out_wr_ptr], USBD_AD_PACKET_SZE);
              }
            }
            else if (LOBYTE(req->wIndex) == 2U)
            {
              haudio->in_alt_setting = req->wValue;
              if (req->wValue == 1U)
              {
                /* Host just switched the mic AudioStreaming interface to
                   its operational alt-setting - this, not USBD_Init(), is
                   the right moment to arm the first IN packet, since the
                   host is now actually scheduling bandwidth and polling
                   this endpoint. See the comment in USBD_AD_Init(). */
                USBD_memset(&haudio->in_buf[haudio->in_rd_ptr], 0, USBD_AD_PACKET_SZE);
                (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, &haudio->in_buf[haudio->in_rd_ptr], USBD_AD_PACKET_SZE);
              }
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

static void AD_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }
  USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
  (void)USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, USB_MAX_EP0_SIZE));
}

static void AD_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }
  if (req->wLength != 0U)
  {
    haudio->control.cmd = 0x01U; /* SET_CUR */
    haudio->control.len = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);
    haudio->control.unit = HIBYTE(req->wIndex);
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
  }
}

/* Diagnostics: any class SET_CUR the host ever sends us, whatever unit -
   in particular whether it ever mutes Feature Unit 2 (the speaker/TX
   path), which some hosts do in hardware when a device advertises a
   Mute Control, independently of the stream's own software volume. */
volatile uint32_t usb_setcur_count      = 0;
volatile uint32_t usb_setcur_last_unit  = 0xFFU;
volatile uint32_t usb_setcur_last_value = 0;

static uint8_t USBD_AD_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  if (haudio->control.cmd == 0x01U)
  {
    usb_setcur_count++;
    usb_setcur_last_unit  = haudio->control.unit;
    usb_setcur_last_value = haudio->control.data[0];

    if (haudio->control.unit == 0x02U /* Feature Unit 2, mute */)
    {
      ((USBD_AUDIO_DUPLEX_ItfTypeDef *)pdev->pUserData[pdev->classId])->MuteCtl(haudio->control.data[0]);
    }
    haudio->control.cmd = 0U;
    haudio->control.len = 0U;
  }
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_AD_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_AD_SOF(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);
  return (uint8_t)USBD_OK;
}

/* Shared between IsoINIncomplete() and DataIn() below - either can be the
   one that crosses into or out of an underrun. Confirmed on hardware
   2026-08-06: usb_mic_underruns telemetry shows the underrun itself is
   small and clean - about 7-8 packets, only right at an RX/TX transition,
   never during steady state either side - so the remaining "crackle" isn't
   the underrun, it's that both of its edges are a hard cut between real
   audio and silence. A step discontinuity in a PCM stream is audible as a
   click regardless of whether the data on either side is individually
   correct, same reason cw_tx_process_block() ramps its envelope with a
   raised cosine rather than switching the tone on/off abruptly. Fading the
   one packet on each side of an underrun - not the underrun itself, which
   stays exactly as short as the guard above already makes it - removes the
   discontinuity without changing how much silence is actually sent. */
static uint8_t in_last_real_pkt[USBD_AD_PACKET_SZE];
static uint8_t in_was_silent = 1U;   /* starts true: nothing sent yet */

/* Scales 16-bit mono PCM (same little-endian layout as in_buf) by a gain
   ramping linearly from 0 up to (n-1)/n (fade_in) or from (n-1)/n down to
   0 (!fade_in) across the packet, and transmits it. Integer math only -
   this runs in interrupt context on every packet at the edge of an
   underrun, and the existing style in this file avoids anything heavier
   than that here. */
static void USBD_AD_MicTransmitRamped(USBD_HandleTypeDef *pdev, const uint8_t *src, uint8_t fade_in)
{
  static uint8_t pkt[USBD_AD_PACKET_SZE];
  const uint32_t n = USBD_AD_PACKET_SZE / 2U;
  uint32_t i;

  for (i = 0U; i < n; i++)
  {
    int16_t  s      = (int16_t)((uint16_t)src[2U * i] | ((uint16_t)src[2U * i + 1U] << 8));
    uint32_t step   = fade_in ? i : (n - 1U - i);
    int32_t  scaled = ((int32_t)s * (int32_t)step) / (int32_t)(n - 1U);

    pkt[2U * i]      = (uint8_t)((uint16_t)scaled & 0xFFU);
    pkt[2U * i + 1U] = (uint8_t)(((uint16_t)scaled >> 8) & 0xFFU);
  }
  (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, pkt, USBD_AD_PACKET_SZE);
}

static uint8_t USBD_AD_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* The HAL's IISOIXFR handling aborts the transfer and *disables* the
     endpoint (HAL_PCD_EP_Abort() in stm32f7xx_hal_pcd.c), so a single
     missed frame would otherwise end the mic stream permanently - DataIn()
     never fires again for a transfer that was killed rather than
     completed. Re-arm here so a dropped frame costs one packet, not the
     whole session. Same underrun guard as DataIn() below - see there for
     why. */
  if (epnum == (USBD_AD_IN_EP & 0x7FU))
  {
    uint16_t avail = (uint16_t)((haudio->in_wr_ptr + USBD_AD_IN_RING_SZE - haudio->in_rd_ptr) % USBD_AD_IN_RING_SZE);

    if (avail >= USBD_AD_PACKET_SZE)
    {
      uint8_t *real = &haudio->in_buf[haudio->in_rd_ptr];

      if (in_was_silent)
      {
        USBD_AD_MicTransmitRamped(pdev, real, 1U);
        in_was_silent = 0U;
        usb_mic_fade_ins++;
      }
      else
      {
        (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, real, USBD_AD_PACKET_SZE);
      }
      for (uint32_t i = 0U; i < USBD_AD_PACKET_SZE; i++)
      {
        in_last_real_pkt[i] = real[i];
      }
      haudio->in_rd_ptr = (uint16_t)((haudio->in_rd_ptr + USBD_AD_PACKET_SZE) % USBD_AD_IN_RING_SZE);
    }
    else
    {
      if (in_was_silent)
      {
        static const uint8_t silence[USBD_AD_PACKET_SZE] = {0};
        (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, (uint8_t *)silence, USBD_AD_PACKET_SZE);
      }
      else
      {
        USBD_AD_MicTransmitRamped(pdev, in_last_real_pkt, 0U);
        in_was_silent = 1U;
        usb_mic_fade_outs++;
      }
      usb_mic_underruns++;
    }
  }

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_AD_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  (void)USBD_LL_PrepareReceive(pdev, epnum, &haudio->out_buf[haudio->out_wr_ptr], USBD_AD_PACKET_SZE);
  return (uint8_t)USBD_OK;
}

/* Speaker (OUT): a fresh packet has arrived from the host. */
static uint8_t USBD_AD_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint16_t got;

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == USBD_AD_OUT_EP)
  {
    got = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);
    if (got > USBD_AD_PACKET_SZE)
    {
      got = USBD_AD_PACKET_SZE;
    }

    /* Single-writer discipline: this ISR owns out_wr_ptr and nothing else.
       It used to also drag out_rd_ptr forward on overrun, but out_rd_ptr
       belongs to the application-side reader
       (USBD_AUDIO_DUPLEX_GetSpeakerAudio(), main loop) - two writers to
       one index raced, and an ISR landing mid-read made the read pointer
       jump, which is audible as crackle. Overrun is now detected and
       resynced by the reader, which can do it safely. */
    {
      /* Ground truth on whether the host is sending real audio at all,
         checked right where packets land - independent of PipeWire
         routing, the cushion gate, or anything else downstream. If this
         never goes non-zero, the problem is definitively "nothing reaches
         the device", not anything in GetSpeakerAudio(). */
      uint16_t j;
      for (j = 0; j < got; j++)
      {
        if (haudio->out_buf[(haudio->out_wr_ptr + j) % USBD_AD_OUT_RING_SZE] != 0U)
        {
          usb_spk_nonzero_pkts++;
          break;
        }
      }
    }
    haudio->out_wr_ptr = (uint16_t)((haudio->out_wr_ptr + got) % USBD_AD_OUT_RING_SZE);
    usb_spk_rx_pkts++;

    (void)USBD_LL_PrepareReceive(pdev, USBD_AD_OUT_EP, &haudio->out_buf[haudio->out_wr_ptr], USBD_AD_PACKET_SZE);
  }

  return (uint8_t)USBD_OK;
}

/* Mic (IN): previous packet finished sending - queue the next one. */
static uint8_t USBD_AD_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* usbd_core hands DataIn() the bare endpoint index (1), not the full IN
     address (0x81) - it even has to OR in 0x80 itself to look the endpoint
     up (see USBD_LL_DataInStage()). Comparing against USBD_AD_IN_EP
     directly is therefore never true, which left the mic stream armed
     exactly once by SET_INTERFACE and never re-armed: the host saw a
     starved isochronous IN stream, spammed "retire_capture_urb" errors,
     and eventually dropped the device. */
  if (epnum == (USBD_AD_IN_EP & 0x7FU))
  {
    /* This ISR is the sole reader of in_rd_ptr and fires roughly every
       1 ms regardless of how often or how unevenly the app side calls
       USBD_AUDIO_DUPLEX_FeedMic() - steady for non-buffered RX modes,
       bursty for FreeDV's "drain as fast as it fills" pattern. Blindly
       transmitting and advancing in_rd_ptr on every call, the way this
       used to work, assumes the writer is always at least a packet
       ahead; whenever it isn't - a burst-vs-steady mismatch right at a
       mode switch, most likely - the reader walks into bytes nothing
       has written yet and just keeps going, since nothing here ever
       pulls it back once it's ahead of in_wr_ptr. Audible as a
       persistent crackle that a one-time pointer reset doesn't fix,
       since the underlying rate mismatch just recreates it. Checking
       availability first and sending silence on a genuine underrun
       mirrors the speaker side's own resync-on-read discipline (see
       DataOut() above), just for the direction where this ISR, not the
       app, is the reader. */
    uint16_t avail = (uint16_t)((haudio->in_wr_ptr + USBD_AD_IN_RING_SZE - haudio->in_rd_ptr) % USBD_AD_IN_RING_SZE);

    if (avail >= USBD_AD_PACKET_SZE)
    {
      uint8_t *real = &haudio->in_buf[haudio->in_rd_ptr];

      if (in_was_silent)
      {
        USBD_AD_MicTransmitRamped(pdev, real, 1U);
        in_was_silent = 0U;
        usb_mic_fade_ins++;
      }
      else
      {
        (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, real, USBD_AD_PACKET_SZE);
      }
      for (uint32_t i = 0U; i < USBD_AD_PACKET_SZE; i++)
      {
        in_last_real_pkt[i] = real[i];
      }
      haudio->in_rd_ptr = (uint16_t)((haudio->in_rd_ptr + USBD_AD_PACKET_SZE) % USBD_AD_IN_RING_SZE);
    }
    else
    {
      if (in_was_silent)
      {
        static const uint8_t silence[USBD_AD_PACKET_SZE] = {0};
        (void)USBD_LL_Transmit(pdev, USBD_AD_IN_EP, (uint8_t *)silence, USBD_AD_PACKET_SZE);
      }
      else
      {
        USBD_AD_MicTransmitRamped(pdev, in_last_real_pkt, 0U);
        in_was_silent = 1U;
        usb_mic_fade_outs++;
      }
      usb_mic_underruns++;
    }
  }

  return (uint8_t)USBD_OK;
}

uint8_t USBD_AUDIO_DUPLEX_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_AUDIO_DUPLEX_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  pdev->pUserData[pdev->classId] = fops;
  return (uint8_t)USBD_OK;
}

void USBD_AUDIO_DUPLEX_FeedMic(USBD_HandleTypeDef *pdev, const int16_t *pcm16, uint32_t n_samples)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint32_t i;
  uint32_t queued, free_bytes, max_samples;

  if (haudio == NULL)
  {
    return;
  }

  /* A single call writing more samples than are actually free in the ring
     wraps in_wr_ptr back past bytes from the SAME call that the IN
     endpoint ISR hasn't sent yet - self-overwriting mid-write, guaranteed
     corruption rather than just lost audio. Confirmed on hardware
     2026-08-06: every buffered/FreeDV RX mode (1600/2400B/700D/700E, all
     BLOCK_SIZE 3840 -> 1920-sample/3840-byte calls) exceeds
     USBD_AD_IN_RING_SZE (1536 bytes) by more than 2x, every single call -
     "just garble" on FreeDV1600 was this, not a subtle timing issue; SSB's
     much smaller 256-sample calls never hit it, which is why this went
     unnoticed through every earlier test. Cap to what's actually free
     right now (not just the ring's total size - some may already be
     queued), same discipline GetSpeakerAudio() already applies on the
     speaker side. The trailing samples are lost same as any other
     underrun - a real but far smaller problem than corrupting what was
     already queued to go out. */
  queued      = (uint32_t)((haudio->in_wr_ptr + USBD_AD_IN_RING_SZE - haudio->in_rd_ptr) % USBD_AD_IN_RING_SZE);
  free_bytes  = USBD_AD_IN_RING_SZE - queued;
  max_samples = free_bytes / 2U;
  if (n_samples > max_samples)
  {
    n_samples = max_samples;
  }

  for (i = 0; i < n_samples; i++)
  {
    haudio->in_buf[haudio->in_wr_ptr]     = (uint8_t)(((uint16_t)pcm16[i]) & 0xFFU);
    haudio->in_buf[haudio->in_wr_ptr + 1U] = (uint8_t)((((uint16_t)pcm16[i]) >> 8) & 0xFFU);
    haudio->in_wr_ptr = (uint16_t)((haudio->in_wr_ptr + 2U) % USBD_AD_IN_RING_SZE);
  }
}

/* Diagnostics for the speaker (OUT) path, read by main.c's telemetry. */
volatile uint32_t usb_spk_calls     = 0;   /* GetSpeakerAudio() invocations */
volatile uint32_t usb_spk_underruns = 0;   /* blocks that came up short */
volatile uint32_t usb_spk_avail     = 0;   /* last observed fill, in samples */
volatile uint32_t usb_spk_avail_min = 0xFFFFFFFFU; /* lowest fill seen since last report - catches
                                                       brief dips the once-a-second "last value" misses */
volatile uint32_t usb_spk_rx_pkts   = 0;   /* OUT packets accepted from the host */
volatile uint32_t usb_spk_nonzero_pkts = 0; /* of those, how many had real (non-zero) content */
volatile uint32_t usb_spk_laps      = 0;   /* times the reader fell behind enough to skip forward */

/* Mic (IN) path: DataIn()/IsoINIncomplete() are the reader here (unlike the
   speaker side, where the app is the reader) - see both below. */
volatile uint32_t usb_mic_underruns = 0;   /* packet sends where nothing new had been written yet */
/* Diagnostic pair added 2026-08-06 chasing a hum reported specifically on
   usb-source activation, alternating every other switch - not explained by
   usb_mic_underruns alone (that only says *how many* packets underran, not
   how many separate fade-out/fade-in *cycles* that was, which is what a
   per-switch ResetMic() should produce exactly one of each time). Counted
   separately from usb_mic_underruns instead of inferred from it so a switch
   that produces an unusually long or repeated fade sequence is visible
   directly, without having to guess it from a raw packet count. */
volatile uint32_t usb_mic_fade_ins  = 0;   /* IN packets sent as a fade-in ramp (silence -> real) */
volatile uint32_t usb_mic_fade_outs = 0;   /* IN packets sent as a fade-out ramp (real -> silence) */

void USBD_AUDIO_DUPLEX_ResetSpeaker(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  /* Called when transmit starts: drop whatever accumulated while nothing
     was consuming, and begin a fresh cushion from the writer's position. */
  haudio->out_rd_enable = 0U;
  haudio->out_rd_ptr    = haudio->out_wr_ptr;
}

void USBD_AUDIO_DUPLEX_ResetMic(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  /* Mic-direction counterpart to ResetSpeaker() above, same idea in
     reverse: the app writes in_wr_ptr, the IN endpoint ISR reads
     in_rd_ptr, so realigning rd to wr drops whatever was queued and has
     the ISR start sending fresh samples from "now" instead of whatever
     was left over from before a mode or source change.

     Root cause found 2026-08-06 for a small, exactly-periodic amplitude
     spike (period == USBD_AD_IN_RING_SZE, once per ring lap) heard only
     over audio:usb, in every mode, that no amount of resetting elsewhere
     ever fixed: in_wr_ptr only stays a multiple of 2 (one sample at a
     time), but the ISR reads USBD_AD_PACKET_SZE *linear* bytes from
     &in_buf[in_rd_ptr] via USBD_LL_Transmit(), which has no concept of
     the ring wrapping - safe only when in_rd_ptr is itself a multiple of
     USBD_AD_PACKET_SZE (every multiple-of-PACKET_SZE offset has a full
     packet's room to the physical end of in_buf, since RING_SZE is
     itself a multiple of PACKET_SZE). Plain `in_rd_ptr = in_wr_ptr`
     lands rd on a non-multiple-of-PACKET_SZE offset almost every time (1
     in 48 odds of landing clean), and that misalignment is permanent
     from then on - in_rd_ptr only ever advances by whole packets, so its
     remainder mod PACKET_SZE never changes again until the next reset
     re-rolls it to some other arbitrary value. Once misaligned, one read
     per lap walks PACKET_SZE bytes past the physical end of in_buf[] and
     into whatever struct field follows it (in_wr_ptr/in_rd_ptr, then
     control) - audible as a small spike (those fields' raw values,
     interpreted as PCM) followed by a run of near-silence (control's
     mostly-zero bytes), recurring like clockwork every RING_SZE bytes.
     Confirmed against a clean, unmodulated capture: exactly-768-sample
     (RING_SZE) spacing, sample-accurate, for as long as the ring stays
     misaligned - which by the above is "since whenever it last got
     reset", i.e. effectively always. Rounding down to the packet grid
     here means every reset - mode switch, PTT, source switch - leaves
     in_rd_ptr aligned, and once aligned it stays aligned forever after,
     since every subsequent advance is a whole multiple of PACKET_SZE. */
  haudio->in_rd_ptr = (uint16_t)(haudio->in_wr_ptr - (haudio->in_wr_ptr % USBD_AD_PACKET_SZE));
}

uint32_t USBD_AUDIO_DUPLEX_GetSpeakerAudio(USBD_HandleTypeDef *pdev, int16_t *pcm16, uint32_t n_samples)
{
  USBD_AUDIO_DUPLEX_HandleTypeDef *haudio = (USBD_AUDIO_DUPLEX_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint32_t i, avail_bytes, avail_samples, got, cushion;

  if (haudio == NULL)
  {
    USBD_memset(pcm16, 0, n_samples * sizeof(int16_t));
    return 0U;
  }

  avail_bytes = (haudio->out_wr_ptr >= haudio->out_rd_ptr)
                  ? (uint32_t)(haudio->out_wr_ptr - haudio->out_rd_ptr)
                  : (uint32_t)(USBD_AD_OUT_RING_SZE - haudio->out_rd_ptr + haudio->out_wr_ptr);
  avail_samples = avail_bytes / 2U;

  usb_spk_calls++;
  usb_spk_avail = avail_samples;
  if (avail_samples < usb_spk_avail_min) usb_spk_avail_min = avail_samples;

  /*
   * The host delivers 48 samples every 1 ms, but this reader consumes a
   * whole DSP block (typically 256 samples) at once every ~5.3 ms, so the
   * fill level swings by a full block either side of its average even
   * when the long-term rates match exactly. Reading the instant the ring
   * happens to hold anything at all therefore underruns on most blocks,
   * and the old code silently zero-padded the shortfall - a burst of
   * silence spliced into the audio every few milliseconds, which is
   * exactly the crackle this fixes.
   *
   * Instead, hold off until a cushion of roughly half the ring has built
   * up before streaming (the same idea as ST's rd_enable gating in its
   * stock speaker class), and on a genuine underrun re-arm that cushion
   * rather than limping along glitching once per block.
   */
  /* Cushion scales with the caller's block size rather than being fixed:
     the analog modes ask for 256 samples at a time, the FreeDV modes for
     1920 (BLOCK_SIZE_MAX/2), so any constant would be either useless for
     one or unaffordable for the other. Half a block is enough to absorb
     the sawtooth without adding needless latency. */
  cushion = n_samples / 2U;

  if (haudio->out_rd_enable == 0U)
  {
    /* Refilling. out_rd_ptr was snapped to the writer when the refill
       started, so avail_samples here is genuinely "how much has arrived
       since then" and grows from zero - it is not a stale distance left
       over from whenever this last ran. That matters because while TX is
       off nothing calls this function at all, yet the USB ISR keeps
       advancing out_wr_ptr right around the ring, so a leftover
       out_rd_ptr would otherwise make avail_samples meaningless on the
       first block of the next transmission. */
    if (avail_samples < (cushion + n_samples))
    {
      USBD_memset(pcm16, 0, n_samples * sizeof(int16_t));
      return 0U;
    }
    haudio->out_rd_enable = 1U;
  }

  if (avail_samples < n_samples)
  {
    /* Underrun: close the gate and wait for the cushion to rebuild, but do
       NOT move out_rd_ptr - re-confirmed correct after a waterfall showed
       the tone genuinely staying at the right frequency, just chopped
       (dropouts), not shifted. Snapping out_rd_ptr forward to out_wr_ptr
       here discards every sample still sitting unplayed in the ring on
       every ordinary underrun, which is real audio loss on top of the
       gap itself. Leaving it alone means the next refill resumes from the
       exact sample it left off at - the gap is still there (that's the
       underrun itself, see the ring size increase below for reducing how
       often it happens), but nothing already received is thrown away. */
    haudio->out_rd_enable = 0U;
    usb_spk_underruns++;
    USBD_memset(pcm16, 0, n_samples * sizeof(int16_t));
    return 0U;
  }

  /* Fell far enough behind that the writer is about to lap us (host clock
     slightly faster, or a long DSP stall). Skip forward to the cushion
     distance behind the writer - done here, in the single owner of
     out_rd_ptr, so it cannot race the USB ISR. This is the opposite case
     from the underrun above (too much unread data, not too little), and
     it discards buffered audio too - but had no telemetry of its own, so
     it could have been firing (and clicking) invisibly to usb_spk_under. */
  if (avail_bytes > (USBD_AD_OUT_RING_SZE - (2U * USBD_AD_PACKET_SZE)))
  {
    haudio->out_rd_ptr = (uint16_t)((haudio->out_wr_ptr + USBD_AD_OUT_RING_SZE
                                     - (cushion * 2U)) % USBD_AD_OUT_RING_SZE);
    usb_spk_laps++;
  }

  got = n_samples;

  for (i = 0; i < got; i++)
  {
    uint16_t lo = haudio->out_buf[haudio->out_rd_ptr];
    uint16_t hi = haudio->out_buf[haudio->out_rd_ptr + 1U];
    pcm16[i] = (int16_t)((hi << 8) | lo);
    haudio->out_rd_ptr = (uint16_t)((haudio->out_rd_ptr + 2U) % USBD_AD_OUT_RING_SZE);
  }

  return got;
}
