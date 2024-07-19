#pragma once

#include <functional>
#include "PacketUtils.h"

class ICommStream
{
public:
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void write(const uint8_t *constdata, size_t length) = 0;
    virtual void write(uint8_t byte) = 0;
    virtual void write(const char *const data, size_t length)
    {
        this->write((uint8_t *)data, length);
    }

    using PacketHandler = std::function<bool(char *, size_t, ICommStream *)>;

    PacketHandler handler;
    bool packetHandlerBound;

    void bindPacketHandler(PacketHandler handler)
    {
        ICommStream::handler = handler;
        packetHandlerBound = true;
    }

protected:
    bool parsePacket(char *packet, size_t length, ICommStream *commStream)
    {
        if (!packetHandlerBound)
            return false;

        return handler(packet, length, commStream);
    }
};