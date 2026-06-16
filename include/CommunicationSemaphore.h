#pragma once
#include "UUID.h"

class CommunicationSemaphore
{
public:
    CommunicationSemaphore(unsigned long timeout);

    // Might wanna do identifier based, since multiple different channels can originate from the same remote...
    bool acquire(const UUID &lockID);
    bool release(const UUID &lockID);

    const bool lockExpired() const;

    const UUID &currentLockHolder() const;

private:
    UUID lockHolder;
    unsigned long lockTime;

    const unsigned long lockExpiry;
};