#pragma once

class Debug
{
public:
    static void write(const char message[]);

    static void writef(const char format[], ...);
};