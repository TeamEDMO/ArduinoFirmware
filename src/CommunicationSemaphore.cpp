#include "CommunicationSemaphore.h"
#include <Arduino.h>

CommunicationSemaphore::CommunicationSemaphore(unsigned long timeout)
    : lockHolder{{0, 0, 0, 0}}, lockTime{0}, lockExpiry(timeout) {}

bool CommunicationSemaphore::acquire(const UUID &lockID)
{
    unsigned long currentTime = millis();

    // We do not yield the lock, if the lock is held by another communication channel, but the timeout has not been exceeded
    if (lockHolder != lockID && currentTime - lockTime < lockExpiry)
        return false;

    lockHolder = lockID;
    lockTime = currentTime;

    return true;
}

bool CommunicationSemaphore::release(const UUID &lockID)
{
    if (lockHolder != lockID)
        return false;

    lockHolder = UUID({0, 0, 0, 0});
    lockTime = 0;
    return true;
}

const bool CommunicationSemaphore::lockExpired() const
{
    unsigned long currentTime = millis();

    return (currentTime - lockTime) >= lockExpiry;
}

const UUID &CommunicationSemaphore::currentLockHolder() const
{
    return lockHolder;
}