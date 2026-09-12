#include"BLE.h"
#include "Arduino.h"

uint32_t sampleCount = 0;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool streamEnable = false;
bool sendData = false;
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer)
    {
        deviceConnected = true;

        Serial.println("BLE DEVICE CONNECTED");
    }

    void onDisconnect(BLEServer* pServer)
    {
        deviceConnected = false;

        Serial.println("BLE DEVICE DISCONNECTED");

        BLEDevice::startAdvertising();
    }
};
class MyCallbacks: public BLECharacteristicCallbacks
{
    
  void onWrite(BLECharacteristic *pCharacteristic)
    {
        // 1. Get the raw data and its length
        uint8_t* rawData = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();

        Serial.print("Received bytes: ");
        for (int i = 0; i < length; i++) {
            Serial.printf("%02X ", rawData[i]); // Prints as readable HEX (e.g., "AA 01 AA")
        }
        Serial.println();

        // 2. Check length first to avoid crashes
        if (length >= 3) 
        {
            // ================= START (AA 01 AA) =================
            if (rawData[0] == 0xAA && rawData[1] == 0x01 && rawData[2] == 0xAA)
            {
                Serial.println("START CMD RECEIVED");

                uint8_t ackPacket[] = {0xCC, 0x01, 0xCC};
                pTxCharacteristic->setValue(ackPacket, 3);
                pTxCharacteristic->notify();
                sendData = true;
								 sampleCount = 0;

            }
            // ================= STOP (AA 02 AA) =================
            else if (rawData[0] == 0xAA && rawData[1] == 0x02 && rawData[2] == 0xAA)
            {
                sendData = false;
								sampleCount = 0;
                Serial.println("STOP CMD RECEIVED");

                uint8_t ackPacket[] = {0xCC, 0x02, 0xCC};
                pTxCharacteristic->setValue(ackPacket, 3);
                pTxCharacteristic->notify();
            }
            else 
            {
                sendInvalidAck();
            }
        }
        else 
        {
            sendInvalidAck();
        }
    }

    // Helper to keep code clean
    void sendInvalidAck() {
        Serial.println("INVALID CMD");
        uint8_t ackPacket[] = {0xCC, 0x00, 0xCC};
        pTxCharacteristic->setValue(ackPacket, 3);
        pTxCharacteristic->notify();
    }
};


void BLE_Init()
{
          // ================= BLE =================
 BLEDevice::init("BP RIGHT");                                                         //////////////////device name

pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

BLEService *pService =
    pServer->createService(SERVICE_UUID);

// TX
pTxCharacteristic =
    pService->createCharacteristic(
        CHARACTERISTIC_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );

pTxCharacteristic->addDescriptor(new BLE2902());

// RX
pRxCharacteristic =
    pService->createCharacteristic(
        CHARACTERISTIC_RX,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );

pRxCharacteristic->setCallbacks(new MyCallbacks());

pService->start();

BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

pAdvertising->addServiceUUID(SERVICE_UUID);

pAdvertising->setScanResponse(true);

pAdvertising->start();
}