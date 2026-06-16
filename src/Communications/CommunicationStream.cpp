#include "Communications/CommunicationStream.h"

CommunicationStream::CommunicationStream() : identifier{} {}

void CommunicationStream::write(const char *const data, size_t length)
{
    this->write((uint8_t *)data, length);
}

void CommunicationStream::bindPacketHandler(PacketHandler handler)
{
    CommunicationStream::handler = handler;
    packetHandlerBound = true;
}

void CommunicationStream::parsePacket(char *packet, size_t length, CommunicationStream *commStream)
{
    if (!packetHandlerBound)
        return;

    handler(packet, length, commStream);
}