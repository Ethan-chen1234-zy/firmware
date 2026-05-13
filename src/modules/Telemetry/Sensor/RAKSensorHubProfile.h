#pragma once

#if defined(HAS_RAKHUB) && defined(RAK_SENSORHUB_USB_PROFILE) && RAK_SENSORHUB_USB_PROFILE

/** XMODEM receive complete (see xmodem.cpp); future: parse uploaded profile JSON/CSV. POC: log only. */
void rakhubNotifyProfileFileUploaded(const char *filename);

#endif
