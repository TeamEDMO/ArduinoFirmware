#include "UUID.h"
#include <Arduino.h>

UUID::UUID()
{
    for (int i = 0; i < 4; ++i)
        _words[i] = random();
}

UUID::UUID(const uint32_t (&words)[4])
{
    for (int i = 0; i < 4; ++i)
        _words[i] = words[i];
}

const uint32_t *UUID::asWords() const
{
    return _words;
}

bool UUID::equals(const UUID &lhs, const UUID &rhs)
{
    for (int i = 0; i < 4; ++i)
    {
        if (lhs._words[i] != rhs._words[i])
            return false;
    }
    return true;
}

bool operator==(const UUID &lhs, const UUID &rhs)
{
    return UUID::equals(lhs, rhs);
}

bool operator!=(const UUID &lhs, const UUID &rhs)
{
    return !UUID::equals(lhs, rhs);
}