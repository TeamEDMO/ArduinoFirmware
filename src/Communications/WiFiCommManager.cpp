#include "Communications/WiFiCommManager.h"
#include "Communications/DummyCommStream.h"

#if WIFI_SUPPORT == 1 && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

#include <WiFi101.h>
#include <WiFiUdp.h>
#include "Communications/WiFiCommStream.h"
#include <unordered_map>
struct WifiCommManager::Impl
{
    WiFiUDP udp{};
    std::unordered_map<uint32_t, WiFiCommStream *> commStreamMapping{};

    WifiCommManager::PacketHandler packetHandler;
    bool packetHandlerBound{};
};

void WifiCommManager::init()
{
    WiFi.setPins(8, 7, 4, 2);
    WiFi.hostname(hostname.c_str());
    WiFi.begin(ssid, pass);

    if (pImpl != nullptr)
        delete pImpl;

    pImpl = new Impl();

    pImpl->udp.begin(2121);
}

void WifiCommManager::bindPacketHandler(PacketHandler handler)
{
    pImpl->packetHandler = handler;
    pImpl->packetHandlerBound = true;
}

void WifiCommManager::update()
{
    int packetSize = pImpl->udp.parsePacket();

    if (packetSize == 0)
        return;

    // No packet handler is available
    // Drop the packet
    if (!pImpl->packetHandlerBound)
        return;

    auto remoteIP = pImpl->udp.remoteIP();
    auto remotePort = pImpl->udp.remotePort();

    char buffer[packetSize];
    pImpl->udp.readBytes(buffer, packetSize);

    auto iterator = pImpl->commStreamMapping.find(remoteIP);

    if (iterator == pImpl->commStreamMapping.end())
    {
        auto commStreamPair = pImpl->commStreamMapping.emplace((u_int32_t)remoteIP, new WiFiCommStream(pImpl->udp, remoteIP, remotePort));
        iterator = commStreamPair.first;

        iterator->second->bindPacketHandler(pImpl->packetHandler);
    }
    iterator->second->updatePort(remotePort);
    iterator->second->receive(packetSize, buffer);
}

void WifiCommManager::PerformOnAllChannels(std::function<void(CommunicationStream *)> action)
{
    for (auto &pair : pImpl->commStreamMapping)
        action(pair.second);
}

CommunicationStream *WifiCommManager::GetChannelWithUUID(const UUID &uuid)
{
    for (auto &pair : pImpl->commStreamMapping)
        if (pair.second->identifier == uuid)
            return pair.second;

    return &dummyComms;
}

#else

void WifiCommManager::init() {}
void WifiCommManager::update() {}
void WifiCommManager::PerformOnAllChannels(std::function<void(CommunicationStream *)> action) {}
void WifiCommManager::bindPacketHandler(PacketHandler handler) {}

CommunicationStream *WifiCommManager::GetChannelWithUUID(const UUID &uuid)
{
    return &dummyComms;
}

#endif

WifiCommManager WifiComms{};