#include "BLE.h"
#include "Arduino.h"
#include "espnow_pairing.h"
#include "nvs_pairing.h"

uint32_t sampleCount = 0;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool streamEnable = false;
bool sendData = false;
uint8_t received_data[BUFFERSIZE] = {0};
uint8_t received_data_len = 0;
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
BLECharacteristic *pRxCharacteristic = NULL;
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;

        Serial.println("BLE DEVICE CONNECTED");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;

        Serial.println("BLE DEVICE DISCONNECTED");

        BLEDevice::startAdvertising();
    }
};
class MyCallbacks : public BLECharacteristicCallbacks
{

    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // 1. Get the raw data and its length
        uint8_t *data = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();
        memcpy(received_data, data, length);
        received_data_len = length;
        Serial.print("Received bytes: ");
        for (int i = 0; i < length; i++)
        {
            Serial.printf("%02X ", received_data[i]); // Prints as readable HEX (e.g., "AA 01 AA")
        }
        Serial.print("\n");
        // 2. Check length first to avoid crashes
        if (length >= 3)
        {
            // // ================= START (AA 01 AA) =================
            // if (received_data[0] == 0xAA && received_data[1] == 0x01 && received_data[2] == 0xBB)
            // {
            //     Serial.println("START CMD RECEIVED");
            //     pTxCharacteristic->setValue(received_data, sizeof(received_data));
            //     pTxCharacteristic->notify();
            //     sendData = true;
            //     sampleCount = 0;
            // }
            // // ================= STOP (AA 02 AA) =================
            // else if (received_data[0] == 0xAA && received_data[1] == 0x02 && received_data[2] == 0xAA)
            // {
            //     sendData = false;
            //     sampleCount = 0;
            //     Serial.println("STOP CMD RECEIVED");
            //     pTxCharacteristic->setValue(received_data, sizeof(received_data));
            //     pTxCharacteristic->notify();
            // }
            // else
            // {
            //     sendInvalidAck(); // invalid command FF FF FF
            // }
            if (received_data[0] == 0xAA && received_data[2] == 0xBB)
            {
                command_exicution(received_data[1]);

                send_data(received_data,received_data_len);
                // Serial.println("vailid data"); // invalid size -AA
            }
            else
            {
                sendInvalidAck(); // invalid command FF FF FF
            }
        }
        else
        {
            Serial.println("INVALID Size"); // invalid size -AA
            uint8_t ackPacket = 0xAA;
            pTxCharacteristic->setValue(&ackPacket, 1);
            pTxCharacteristic->notify();
        }
    }

    // Helper to keep code clean
    void sendInvalidAck()
    {
        Serial.println("INVALID CMD");
        uint8_t error[] = {0xFF, 0xFF, 0xFF};
        pTxCharacteristic->setValue(error, sizeof(error));
        pTxCharacteristic->notify();
    }
};

void command_exicution(uint8_t cmd)
{
    switch (cmd)
    {
    case 0x01:
        Serial.println("START CMD RECEIVED");
        sendData = true;
        sampleCount = 0;
        break;
    case 0x02:
        clearPeerMAC();
        break;
    case 0xEE:
        esp_restart();
        break;
    case 0xFF:
        sendData = false;
        sampleCount = 0;
        Serial.println("STOP CMD RECEIVED");
        break;

    default:
        Serial.println("invalid command");
    }
    pTxCharacteristic->setValue(received_data, received_data_len);
    pTxCharacteristic->notify();
}

void BLE_Init()
{
    // ================= BLE =================
    BLEDevice::init("BP RIGHT"); //////////////////device name

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService =
        pServer->createService(SERVICE_UUID);

    // TX
    pTxCharacteristic =
        pService->createCharacteristic(
            CHARACTERISTIC_TX,
            BLECharacteristic::PROPERTY_NOTIFY);

    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX
    pRxCharacteristic =
        pService->createCharacteristic(
            CHARACTERISTIC_RX,
            BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_WRITE_NR);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(SERVICE_UUID);

    pAdvertising->setScanResponse(true);

    pAdvertising->start();
}