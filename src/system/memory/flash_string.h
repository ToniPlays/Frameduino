#ifndef FLASH_STRING_H
#define FLASH_STRING_H

#include <WString.h>

#define F_STR(x, s) (read_fstring(F(x), s))

namespace Frameduino
{
    inline static const char* read_fstring(const __FlashStringHelper* helper, uint16_t length)
    {
        char buff[length];
        strncpy_P(buff, (const char*)helper, length + 1);
        return buff;
    }
}

#endif