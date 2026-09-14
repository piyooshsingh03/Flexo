#ifndef NVS_PAIRING_H
#define NVS_PAIRING_H

#include <Arduino.h>

bool savePeerMAC(const uint8_t *mac);
bool loadPeerMAC(uint8_t *mac);
bool isPeerMACStored();
void clearPeerMAC();
void check_saved_mac(void);
#endif