#ifndef ESPNOW_PAIRING_H
#define ESPNOW_PAIRING_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// PACKET TYPES
// ============================================================

#define PAIR_REQUEST   1
#define PAIR_RESPONSE  2
#define PAIR_CONFIRM   3
#define DATA_EXCHANGE  4

// ============================================================
// PAIRING PACKET
// ============================================================

typedef struct
{
    uint8_t type;

} PairPacket;

// ============================================================
// DATA PACKET
// ============================================================

typedef struct
{
    uint8_t type;
    uint32_t counter;
} DataPacket;

// ============================================================
// GLOBAL VARIABLES
// ============================================================

extern bool paired;

extern bool responseSent;

extern uint8_t peerMAC[6];

extern unsigned long lastPairRequest;

extern uint32_t txCounter ;
extern unsigned long lastDataSend ;
// ============================================================
// INITIALIZATION
// ============================================================

bool initESPNowPairing();

// ============================================================
// PAIRING FUNCTIONS
// ============================================================

void sendPairRequest();

void sendPairResponse(const uint8_t *mac);

void sendPairConfirm();

void loadSavedPair();

// ============================================================
// PEER MANAGEMENT
// ============================================================

bool addPeer(const uint8_t *mac);

// ============================================================
// UTILITY
// ============================================================

void printMAC(const uint8_t *mac);

// ============================================================
// ESP-NOW CALLBACKS
// ============================================================

void OnDataSent(
    const uint8_t *mac_addr,
    esp_now_send_status_t status
);

void OnDataRecv(
    const uint8_t *mac,
    const uint8_t *data,
    int len
);

void send_Data(uint16_t data);
void sendCounter();
void espnow_setup(void);
void send_data(uint8_t *pdata,uint8_t len);
#endif