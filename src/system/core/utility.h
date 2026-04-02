#ifndef FRAMEDUINO_CORE_UTILITY_H
#define FRAMEDUINO_CORE_UTILITY_H

#include "system/core/hal_system.h"
#include <Wire.h>

namespace Frameduino
{
    inline static void hal_i2c_scan(TwoWire *wire)
    {
        hal_logger_log_i("scanning I2C devices");

        uint8_t nDevices = 0;
        for (uint8_t address = 1; address < 127; address++)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            wire->beginTransmission(address);
            uint8_t error = wire->endTransmission();
            

            if (error == 0)
            {
                char addr[64];
                sprintf(addr, "Device found at address %02x", address);
                hal_logger_log_i(addr);

                nDevices++;
            }
            else if (error == 4)
            {
                char addr[64];
                sprintf(addr, "Device found at address %02x", address);
                hal_logger_log_i(addr);
            }
        }

        if (nDevices == 0)
            hal_logger_log_w("No I2C devices found\n");
        else
            hal_logger_log_i("done\n");
    }
}

#endif