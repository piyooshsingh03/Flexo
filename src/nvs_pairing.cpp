#include "nvs_pairing.h"
#include "espnow_pairing.h"
#include <Preferences.h>

Preferences preferences;

#define NVS_NAMESPACE "pairing"
#define NVS_KEY       "peer_mac"


void check_saved_mac(void)
{
     // --------------------------------------------------------
    // Check saved pairing
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "Checking saved pairing..."
    );

    loadSavedPair();
    // clearPeerMAC();
}

bool savePeerMAC(const uint8_t *mac)
{
    preferences.begin(NVS_NAMESPACE, false);

    char macString[18];

    snprintf(macString,
             sizeof(macString),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],
             mac[1],
             mac[2],
             mac[3],
             mac[4],
             mac[5]);

    preferences.putString(NVS_KEY, macString);

    preferences.end();

    Serial.print("Saved peer MAC to NVS: ");
    Serial.println(macString);

    return true;
}


bool loadPeerMAC(uint8_t *mac)
{
    if (mac == nullptr)
    {
        return false;
    }

    if (!preferences.begin(NVS_NAMESPACE, true))
    {
        Serial.println("NVS open failed");
        return false;
    }

    // Check whether key exists
    if (!preferences.isKey(NVS_KEY))
    {
        preferences.end();

        Serial.println("No saved MAC found");

        return false;
    }

    // Check stored size
    size_t length = preferences.getBytesLength(NVS_KEY);

    if (length != 6)
    {
        preferences.end();

        Serial.println("Invalid MAC data in NVS");

        return false;
    }

    // Read 6 bytes
    size_t read = preferences.getBytes(
        NVS_KEY,
        mac,
        6
    );

    preferences.end();

    if (read != 6)
    {
        Serial.println("MAC read failed");

        return false;
    }

    Serial.print("MAC loaded: ");

    for (int i = 0; i < 6; i++)
    {
        if (i > 0)
            Serial.print(":");

        if (mac[i] < 0x10)
            Serial.print("0");

        Serial.print(mac[i], HEX);
    }

    Serial.println();

    return true;
}



bool isPeerMACStored()
{
    preferences.begin(NVS_NAMESPACE, true);

    bool exists = preferences.isKey(NVS_KEY);

    preferences.end();

    return exists;
}


void clearPeerMAC()
{
    preferences.begin(NVS_NAMESPACE, false);

    preferences.remove(NVS_KEY);

    preferences.end();

    Serial.println("Peer MAC removed from NVS");
}


