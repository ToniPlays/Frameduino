#ifndef FLASH_STRING_H
#define FLASH_STRING_H

#include ""


namespace Frameduino
{
    inline static const char* read_fstring(const __FlashStringHelper* helper)
    {
        return pgm_read_word(helper);
    }
}

#endif