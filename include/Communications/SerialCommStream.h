#pragma once
#include "CommunicationStream.h"

class SerialCommStream : virtual public CommunicationStream
{
private:
    char dataBuffer[512];
    size_t dataBufferLength = 0;

    bool receivingData = false;

public:
    SerialCommStream();
    void init() override;

    void write(const uint8_t *const data, size_t length) override;

    void write(uint8_t byte) override;
    void begin() override;
    void end() override;

    void update() override;
};

extern SerialCommStream SerialComms;