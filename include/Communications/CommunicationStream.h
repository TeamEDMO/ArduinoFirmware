#pragma once

#include <functional>
#include "../UUID.h"

class CommunicationStream
{
public:
    CommunicationStream();

    UUID identifier;

    virtual void init() = 0;
    virtual void update() = 0;
    virtual void write(const uint8_t *constdata, size_t length) = 0;
    virtual void write(uint8_t byte) = 0;
    virtual void write(const char *const data, size_t length);

    virtual void begin() = 0;
    virtual void end() = 0;

    using PacketHandler = std::function<void(char *, size_t, CommunicationStream *)>;

    void bindPacketHandler(PacketHandler handler);

protected:
    void parsePacket(char *packet, size_t length, CommunicationStream *commStream);

private:
    PacketHandler handler;
    bool packetHandlerBound;
};