#pragma once

#if defined(HAS_RAKHUB) && defined(RAK_SENSORHUB_USB_PROFILE) && RAK_SENSORHUB_USB_PROFILE

/** XMODEM receive complete (see xmodem.cpp); future: parse uploaded profile JSON/CSV. POC: log only. */
void rakhubNotifyProfileFileUploaded(const char *filename);

#if defined(RAK_SENSORHUB_DOWNLINK_POC) && RAK_SENSORHUB_DOWNLINK_POC
/** Bytes SerialConsole discarded while hunting protobuf START1 (0x94). LF ends a RAKHUB line. */
void rakhubUsbFeedByte(uint8_t c);
#endif

#endif
