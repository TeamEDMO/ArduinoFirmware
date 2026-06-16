#pragma once

#include "../globals.h"
#include <functional>

#include "Communications/CommunicationStream.h"
class UUID;

class WifiCommManager
{
public:
    using PacketHandler = CommunicationStream::PacketHandler;

    void init();

    void bindPacketHandler(PacketHandler handler);
    void update();
    void PerformOnAllChannels(std::function<void(CommunicationStream *)> action);

    CommunicationStream *GetChannelWithUUID(const UUID &uuid);

private:
    struct Impl;

    Impl *pImpl;
};

extern WifiCommManager WifiComms;
