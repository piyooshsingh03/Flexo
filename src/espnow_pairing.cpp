#include "espnow_pairing.h"
#include "nvs_pairing.h"

// ============================================================
// GLOBAL VARIABLES
// ============================================================

bool paired = false;

bool responseSent = false;

uint8_t peerMAC[6] = {0};

unsigned long lastPairRequest = 0;
uint32_t txCounter = 0;
unsigned long lastDataSend = 0;
// ============================================================
// BROADCAST MAC
// ============================================================

static uint8_t broadcastAddress[] =
    {
        0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF};
void espnow_setup(void)
{
    Serial.println();
    Serial.println("==============================");
    Serial.println("ESP-NOW PAIRING TEST");
    Serial.println("==============================");

    // --------------------------------------------------------
    // Initialize ESP-NOW
    // --------------------------------------------------------

    if (!initESPNowPairing())
    {
        Serial.println(
            "ESP-NOW setup failed");

        return;
    }

    Serial.println();
    Serial.println(
        "Waiting for another ESP32...");
}
// ============================================================
// PRINT MAC
// ============================================================

void printMAC(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
    {
        if (i > 0)
            Serial.print(":");

        if (mac[i] < 0x10)
            Serial.print("0");

        Serial.print(mac[i], HEX);
    }
}

// ============================================================
// SEND CALLBACK
// ============================================================

void OnDataSent(
    const uint8_t *mac_addr,
    esp_now_send_status_t status)
{
    Serial.print("Send status: ");

    if (status == ESP_NOW_SEND_SUCCESS)
        Serial.println("SUCCESS");
    else
        Serial.println("FAIL");
}

// ============================================================
// ADD PEER
// ============================================================

bool addPeer(const uint8_t *mac)
{
    // --------------------------------------------------------
    // Check if peer already exists
    // --------------------------------------------------------

    if (esp_now_is_peer_exist(mac))
    {
        return true;
    }

    // --------------------------------------------------------
    // Create peer information
    // --------------------------------------------------------

    esp_now_peer_info_t peerInfo = {};

    memcpy(
        peerInfo.peer_addr,
        mac,
        6);

    peerInfo.channel = 0;

    peerInfo.encrypt = false;

    // --------------------------------------------------------
    // Add peer
    // --------------------------------------------------------

    esp_err_t result =
        esp_now_add_peer(&peerInfo);

    if (result == ESP_OK)
    {
        Serial.print("Peer added: ");

        printMAC(mac);

        Serial.println();

        return true;
    }

    Serial.print(
        "Failed to add peer. Error: ");

    Serial.println(result);

    return false;
}

// ============================================================
// SEND PAIR REQUEST
// ============================================================

void sendPairRequest()
{
    PairPacket packet;

    packet.type = PAIR_REQUEST;

    Serial.println();
    Serial.println("==============================");
    Serial.println("Sending PAIR_REQUEST");
    Serial.println("==============================");

    esp_err_t result =
        esp_now_send(
            broadcastAddress,
            (uint8_t *)&packet,
            sizeof(packet));

    if (result == ESP_OK)
    {
        Serial.println(
            "PAIR_REQUEST sent");
    }
    else
    {
        Serial.println(
            "PAIR_REQUEST failed");
    }
}

// ============================================================
// SEND PAIR RESPONSE
// ============================================================

void sendPairResponse(
    const uint8_t *mac)
{
    PairPacket packet;

    packet.type = PAIR_RESPONSE;

    Serial.println();
    Serial.println(
        "Sending PAIR_RESPONSE...");

    esp_err_t result =
        esp_now_send(
            mac,
            (uint8_t *)&packet,
            sizeof(packet));

    if (result == ESP_OK)
    {
        Serial.println(
            "PAIR_RESPONSE queued");
    }
    else
    {
        Serial.println(
            "PAIR_RESPONSE failed");
    }
}

// ============================================================
// SEND PAIR CONFIRM
// ============================================================

void sendPairConfirm()
{
    PairPacket packet;

    packet.type = PAIR_CONFIRM;

    Serial.println();
    Serial.println(
        "Sending PAIR_CONFIRM...");

    esp_err_t result =
        esp_now_send(
            peerMAC,
            (uint8_t *)&packet,
            sizeof(packet));

    if (result == ESP_OK)
    {
        Serial.println(
            "PAIR_CONFIRM queued");
    }
    else
    {
        Serial.println(
            "PAIR_CONFIRM failed");
    }
}

// ============================================================
// LOAD SAVED PAIR
// ============================================================

void loadSavedPair()
{
    uint8_t savedMAC[6];

    // --------------------------------------------------------
    // Check NVS
    // --------------------------------------------------------

    if (!loadPeerMAC(savedMAC))
    {
        Serial.println(
            "No saved pairing found.");

        paired = false;

        return;
    }

    // --------------------------------------------------------
    // Saved MAC found
    // --------------------------------------------------------

    Serial.println();
    Serial.println("==============================");
    Serial.println("SAVED PEER FOUND");
    Serial.println("==============================");

    Serial.print(
        "Peer MAC: ");

    printMAC(savedMAC);

    Serial.println();

    // --------------------------------------------------------
    // Copy saved MAC
    // --------------------------------------------------------

    memcpy(
        peerMAC,
        savedMAC,
        6);

    // --------------------------------------------------------
    // Add saved peer
    // --------------------------------------------------------

    if (addPeer(peerMAC))
    {
        Serial.println(
            "Saved peer added to ESP-NOW");

        paired = true;
    }
    else
    {
        Serial.println(
            "Failed to add saved peer");

        paired = false;
    }
}

// ============================================================
// RECEIVE CALLBACK
// ============================================================

void OnDataRecv(
    const uint8_t *mac,
    const uint8_t *data,
    int len)
{
    // --------------------------------------------------------
    // Check packet length
    // --------------------------------------------------------

    // if (len != sizeof(PairPacket))
    // {
    //     Serial.println(
    //         "Unknown packet received");

    //     return;
    // }

    PairPacket packet;

    memcpy(
        &packet,
        data,
        sizeof(PairPacket));

    // ========================================================
    // PAIR REQUEST
    // ========================================================

    if (packet.type == PAIR_REQUEST)
    {
        Serial.println();
        Serial.println("==============================");
        Serial.println("PAIR_REQUEST RECEIVED");
        Serial.println("==============================");

        Serial.print(
            "Request from: ");

        printMAC(mac);

        Serial.println();

        // ----------------------------------------------------
        // If already paired with SAME device
        // ----------------------------------------------------

        if (paired)
        {
            if (memcmp(peerMAC, mac, 6) == 0)
            {
                Serial.println(
                    "Known partner - reconnecting");

                addPeer(peerMAC);

                return;
            }

            // ------------------------------------------------
            // Different device
            // ------------------------------------------------

            Serial.println(
                "Already paired with another device");

            return;
        }

        // ----------------------------------------------------
        // Save candidate MAC
        // ----------------------------------------------------

        memcpy(
            peerMAC,
            mac,
            6);

        Serial.print(
            "Candidate peer: ");

        printMAC(peerMAC);

        Serial.println();

        // ----------------------------------------------------
        // Add candidate
        // ----------------------------------------------------

        if (!addPeer(peerMAC))
        {
            Serial.println(
                "Could not add candidate peer");

            return;
        }

        // ----------------------------------------------------
        // Send response
        // ----------------------------------------------------

        sendPairResponse(peerMAC);

        responseSent = true;
    }

    // ========================================================
    // PAIR RESPONSE
    // ========================================================

    else if (packet.type == PAIR_RESPONSE)
    {
        Serial.println();
        Serial.println("==============================");
        Serial.println("PAIR_RESPONSE RECEIVED");
        Serial.println("==============================");

        Serial.print(
            "Response from: ");

        printMAC(mac);

        Serial.println();

        // ----------------------------------------------------
        // Already paired
        // ----------------------------------------------------

        if (paired)
        {
            Serial.println(
                "Already paired");

            return;
        }

        // ----------------------------------------------------
        // Save peer MAC
        // ----------------------------------------------------

        memcpy(
            peerMAC,
            mac,
            6);

        Serial.print(
            "Peer MAC: ");

        printMAC(peerMAC);

        Serial.println();

        // ----------------------------------------------------
        // Add peer
        // ----------------------------------------------------

        if (!addPeer(peerMAC))
        {
            Serial.println(
                "Failed to add peer");

            return;
        }

        // ----------------------------------------------------
        // Send confirmation
        // ----------------------------------------------------

        sendPairConfirm();

        // ----------------------------------------------------
        // SAVE PEER TO NVS
        // ----------------------------------------------------

        if (savePeerMAC(peerMAC))
        {
            Serial.println(
                "Peer MAC saved to NVS");
        }
        else
        {
            Serial.println(
                "ERROR: Failed to save peer MAC");
        }

        // ----------------------------------------------------
        // Mark paired
        // ----------------------------------------------------

        paired = true;

        Serial.println();
        Serial.println("==============================");
        Serial.println("PAIRING COMPLETE");
        Serial.println("==============================");
    }

    // ========================================================
    // PAIR CONFIRM
    // ========================================================

    else if (packet.type == PAIR_CONFIRM)
    {
        Serial.println();
        Serial.println("==============================");
        Serial.println("PAIR_CONFIRM RECEIVED");
        Serial.println("==============================");

        Serial.print(
            "Confirmed by: ");

        printMAC(mac);

        Serial.println();

        // ----------------------------------------------------
        // Already paired?
        // ----------------------------------------------------

        if (paired)
        {
            Serial.println(
                "Already paired");

            return;
        }

        // ----------------------------------------------------
        // Save peer MAC
        // ----------------------------------------------------

        memcpy(
            peerMAC,
            mac,
            6);

        // ----------------------------------------------------
        // Add peer
        // ----------------------------------------------------

        if (!addPeer(peerMAC))
        {
            Serial.println(
                "Failed to add peer");

            return;
        }

        // ----------------------------------------------------
        // SAVE PEER TO NVS
        // ----------------------------------------------------

        if (savePeerMAC(peerMAC))
        {
            Serial.println(
                "Peer MAC saved to NVS");
        }
        else
        {
            Serial.println(
                "ERROR: Failed to save peer MAC");
        }

        // ----------------------------------------------------
        // Mark paired
        // ----------------------------------------------------

        paired = true;

        Serial.println();
        Serial.println("==============================");
        Serial.println("PAIRING COMPLETE");
        Serial.println("==============================");

        Serial.print(
            "Partner MAC: ");

        printMAC(peerMAC);

        Serial.println();
    }
    // ============================================================
    // COUNTER DATA
    // ============================================================
    DataPacket pack;

    pack.type = DATA_EXCHANGE;
    pack.counter = txCounter;
    if (packet.type == DATA_EXCHANGE)
    {
        if (len != sizeof(DataPacket))
        {
            Serial.println("Invalid DATA_COUNTER packet");

            return;
        }

        DataPacket packet;

        memcpy(
            &packet,
            data,
            sizeof(DataPacket));

        Serial.println();
        Serial.println("==============================");
        Serial.println("DATA RECEIVED");
        Serial.println("==============================");

        Serial.print("From: ");
        printMAC(mac);

        Serial.println();

        Serial.print("Counter: ");
        Serial.println(packet.counter);

        return;
    }
}

// ============================================================
// INITIALIZE ESP-NOW
// ============================================================

bool initESPNowPairing()
{
    // --------------------------------------------------------
    // WiFi STA mode
    // --------------------------------------------------------

    WiFi.mode(WIFI_STA);

    delay(100);

    Serial.print(
        "My MAC: ");

    Serial.println(
        WiFi.macAddress());

    // --------------------------------------------------------
    // Initialize ESP-NOW
    // --------------------------------------------------------

    if (esp_now_init() != ESP_OK)
    {
        Serial.println(
            "ESP-NOW initialization FAILED");

        return false;
    }

    Serial.println(
        "ESP-NOW initialized");

    // --------------------------------------------------------
    // Register callbacks
    // --------------------------------------------------------

    esp_now_register_send_cb(
        OnDataSent);

    esp_now_register_recv_cb(
        OnDataRecv);

    // --------------------------------------------------------
    // Add broadcast peer
    // --------------------------------------------------------

    esp_now_peer_info_t broadcastPeer = {};

    memcpy(
        broadcastPeer.peer_addr,
        broadcastAddress,
        6);

    broadcastPeer.channel = 0;

    broadcastPeer.encrypt = false;

    if (esp_now_add_peer(&broadcastPeer) != ESP_OK)
    {
        Serial.println(
            "Failed to add broadcast peer");

        return false;
    }

    Serial.println(
        "Broadcast peer added");

    return true;
}

// ============================================================
// SEND data to paired mac
// ============================================================

void send_Data(uint16_t data)
{
    esp_err_t result =
        esp_now_send(
            peerMAC,
            (uint8_t *)&data,
            sizeof(data));

    if (result == ESP_OK)
    {
        Serial.println(
            "data sent : " + String(data));
    }
    else
    {
        Serial.println(
            "data send failed");
    }
}

void sendCounter()
{
    if (!paired)
    {
        return;
    }

    DataPacket pack;

    pack.type = DATA_EXCHANGE;
    pack.counter = txCounter;

    esp_err_t result = esp_now_send(
        peerMAC,
        (uint8_t *)&pack,
        sizeof(pack));

    Serial.println();
    Serial.println("==============================");
    Serial.println("SENDING DATA");
    Serial.println("==============================");

    Serial.print("Counter: ");
    Serial.println(pack.counter);

    if (result == ESP_OK)
    {
        Serial.println("Data sent successfully");
    }
    else
    {
        Serial.print("Send failed, error: ");
        Serial.println(result);
    }

    txCounter++;
}

void send_data(uint8_t *pdata, uint8_t len)
{
    esp_err_t result = esp_now_send(peerMAC,pdata, len);

    if (result == ESP_OK)
    {
        Serial.println("Data sent successfully");
    }
    else
    {
        Serial.print("Send failed, error: ");
        Serial.println(result);
    }
}