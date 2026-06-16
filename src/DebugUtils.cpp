#include "DebugUtils.h"

void Debug::write(const char message[])
{
#ifdef DEBUG
    if (!Serial)
    {
        Serial.begin(115200);

        while (!Serial)
        {
        }
    }

    Serial.println(message);
#endif
}

void Debug::writef(const char format[], ...)
{
#ifdef DEBUG
    va_list ap;
    va_start(ap, format);
    Serial.printf(format, ap);
    va_end(ap);
#endif
}