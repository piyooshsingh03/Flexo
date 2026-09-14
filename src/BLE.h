#ifndef BLE_H_
#define BLE_H_
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_log.h"
#include "esp_system.h"
#define BUFFERSIZE 50
extern uint32_t sampleCount;

extern BLEServer* pServer;
extern BLECharacteristic* pTxCharacteristic ;
extern BLECharacteristic* pRxCharacteristic;
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern bool streamEnable;
extern bool sendData;

extern uint8_t received_data[BUFFERSIZE];
extern uint8_t received_data_len;

// UART Style BLE UUIDs
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

void BLE_Init();
void command_exicution(uint8_t cmd);

#endif