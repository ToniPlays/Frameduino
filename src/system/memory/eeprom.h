#ifndef EEPROM_H
#define EEPROM_H

#include <EEPROM.h>

namespace Frameduino
{
    inline static void hal_eeprom_read(void* target, uint32_t address, uint32_t size)
    {
        eeprom_read_block(target, (const void*)address, size);
    }
    inline static void hal_eeprom_write(uint16_t address, void* data, uint32_t size)
    {
        //eeprom_update_block(&data, (void*)address, size);
    }
}

#endif