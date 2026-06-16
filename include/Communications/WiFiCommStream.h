#pragma once

#include "../globals.h"
#include "CommunicationStream.h"

#if WIFI_SUPPORT == 1

#include <WiFi101.h>
#include <WiFiUdp.h>
class WiFiCommStream : public virtual CommunicationStream
{
private:
    struct Impl;

    Impl *pImpl;

    WiFiCommStream(WiFiUDP &host, IPAddress remoteIP, u_int16_t remotePort);

public:
    void init();

    void write(const uint8_t *const data, size_t length) override;

    void write(uint8_t byte) override;

    void begin() override;

    void end() override;

    void receive(int packetSize, char *packetBuffer);
    void updatePort(uint16_t port);

    void update() override;

    friend class WifiCommManager;
};
#else
#include "DummyCommStream.h"

using WiFiCommStream = DummyCommStream;
#endif