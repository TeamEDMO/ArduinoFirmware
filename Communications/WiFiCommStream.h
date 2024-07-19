#pragma once

#include "../globals.h"
#include <WiFi101.h>
#include <WiFiUdp.h>

#include "ICommStream.h"

#if WIFI_SUPPORT == 1
class WiFiCommStream : public virtual ICommStream
{
private:
    WiFiUDP udp{};

public:
    void init() override
    {
        WiFi.setPins(8, 7, 4, 2);
        WiFi.hostname(hostname.c_str());
        WiFi.begin(ssid, pass);

        udp.begin(2121);
    }

    void write(const uint8_t *const data, size_t length) override
    {
        udp.write(data, length);
    }

    void write(uint8_t byte) override
    {
        udp.write(byte);
    }

    void update() override
    {
        int packetSize = udp.parsePacket();

        if (packetSize == 0)
            return;

        auto remoteIP = udp.remoteIP();
        auto remotePort = udp.remotePort();

        char packetBuffer[packetSize];

        int length = udp.read(packetBuffer, packetSize);

        if (!buffcmp(packetBuffer, commHeader, 2) || !buffcmp(packetBuffer + length - 2, commFooter, 2))
            return;

        udp.beginPacket(remoteIP, remotePort);

        // No reply, don't send a packet
        if (!parsePacket(packetBuffer, packetSize, this))
            return;

        udp.endPacket();
    }
};
#else
// A dummied out version of WiFiCommStream
class WiFiCommStream : public virtual ICommStream
{
public:
    void init() override
    {
    }

    void write(const char *const data, size_t length) override
    {
    }

    void update() override
    {
    }
};
#endif

WiFiCommStream WifiComms;