#pragma once
#include <cstdint>

// Weak expectation, most uses of UUID will be transferred over the wire
class UUID
{
public:
    UUID();

    UUID(const uint32_t (&words)[4]);

    const uint32_t *asWords() const;

    static bool equals(const UUID &lhs, const UUID &rhs);

private:
    uint32_t _words[4];
};

bool operator==(const UUID &lhs, const UUID &rhs);
bool operator!=(const UUID &lhs, const UUID &rhs);