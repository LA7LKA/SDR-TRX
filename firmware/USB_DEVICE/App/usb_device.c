#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio_duplex.h"
#include "usbd_audio_if.h"
#include "main.h"

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, USBD_AUDIO_DUPLEX_CLASS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_AUDIO_DUPLEX_RegisterInterface(&hUsbDeviceFS, &USBD_AUDIO_IF_fops) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }
}
