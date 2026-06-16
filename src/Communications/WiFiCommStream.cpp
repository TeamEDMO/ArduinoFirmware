#include "Communications/WiFiCommStream.h"

#if WIFI_SUPPORT == 1 && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
#include "Communications/PacketUtils.h"
struct WiFiCommStream::Impl
{
    Impl(WiFiUDP &host, IPAddress remoteIP, u_int16_t remotePort) : udp{host}, remoteIP{remoteIP}, remotePort{remotePort} {}

    WiFiUDP &udp;
    const IPAddress remoteIP;
    uint16_t remotePort;
};

WiFiCommStream::WiFiCommStream(WiFiUDP &host, IPAddress remoteIP, u_int16_t remotePort)
    : pImpl{new Impl(host, remoteIP, remotePort)} {}

void WiFiCommStream::init() {}

void WiFiCommStream::write(const uint8_t *const data, size_t length)
{
    pImpl->udp.write(data, length);
}

void WiFiCommStream::write(uint8_t byte)
{
    pImpl->udp.write(byte);
}

void WiFiCommStream::begin()
{
    pImpl->udp.beginPacket(pImpl->remoteIP, pImpl->remotePort);
}

void WiFiCommStream::end()
{
    pImpl->udp.endPacket();
}

void WiFiCommStream::receive(int packetSize, char *packetBuffer)
{
    if (!buffcmp(packetBuffer, commHeader, 2) || !buffcmp(packetBuffer + packetSize - 2, commFooter, 2))
        return;

    parsePacket(packetBuffer, packetSize, this);
}

void WiFiCommStream::updatePort(uint16_t port)
{
    pImpl->remotePort = port;
}

void WiFiCommStream::update() {}

#else

struct WiFiCommStream::Impl
{
};

WiFiCommStream::WiFiCommStream(WiFiUDP &host, IPAddress remoteIP, u_int16_t remotePort) {}

void WiFiCommStream::init() {}

void WiFiCommStream::write(const uint8_t *const data, size_t length) {}

void WiFiCommStream::write(uint8_t byte) {}

void WiFiCommStream::begin() {}

void WiFiCommStream::end() {}

void WiFiCommStream::receive(int packetSize, char *packetBuffer) {}

void WiFiCommStream::updatePort(uint16_t port) {}

void

#endif